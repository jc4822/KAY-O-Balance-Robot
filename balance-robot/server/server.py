from __future__ import annotations

import os, time
from decimal import Decimal
from enum import Enum
from typing import Any, List, Optional

import boto3
from boto3.dynamodb.conditions import Key
from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from fastapi.responses import HTMLResponse


ROBOT_ID     = "KAY-O"
TABLE_NAME   = os.getenv("ROBOTLOG_TABLE", "RobotLogs")
IMAGE_TABLE  = "ImageLogs"
REGION       = os.getenv("AWS_REGION", "eu-west-2")
TTL_DAYS     = int(os.getenv("TTL_DAYS", 30))
ENDPOINT     = os.getenv("DDB_ENDPOINT_URL")

ddb_kwargs: dict[str, Any] = {"region_name": REGION}
if ENDPOINT:
    ddb_kwargs["endpoint_url"] = ENDPOINT
dynamodb = boto3.resource("dynamodb", **ddb_kwargs)
client   = boto3.client("dynamodb",  **ddb_kwargs)


class Action(str, Enum):
    FORWARD  = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT     = "LEFT"
    RIGHT    = "RIGHT"
    STOP     = "STOP"

VALID_MODES = {"AUTO", "MANUAL"}


def create_table_if_missing() -> None:
    try:
        dynamodb.Table(TABLE_NAME).load()
        return
    except client.exceptions.ResourceNotFoundException:
        tbl = dynamodb.create_table(
            TableName=TABLE_NAME,
            BillingMode="PAY_PER_REQUEST",
            KeySchema=[
                {"AttributeName": "RobotId", "KeyType": "HASH"},
                {"AttributeName": "LogTS",  "KeyType": "RANGE"},
            ],
            AttributeDefinitions=[
                {"AttributeName": "RobotId", "AttributeType": "S"},
                {"AttributeName": "LogTS",   "AttributeType": "N"},
            ],
        )
        tbl.wait_until_exists()
        client.update_time_to_live(
            TableName=TABLE_NAME,
            TimeToLiveSpecification={"Enabled": True, "AttributeName": "TTL"},
        )
        print(f"✓ Created {TABLE_NAME} (TTL enabled)")

def create_image_table_if_missing() -> None:
    try:
        dynamodb.Table(IMAGE_TABLE).load()
        return
    except client.exceptions.ResourceNotFoundException:
        tbl = dynamodb.create_table(
            TableName=IMAGE_TABLE,
            BillingMode="PAY_PER_REQUEST",
            KeySchema=[
                {"AttributeName": "Timestamp", "KeyType": "HASH"},
            ],
            AttributeDefinitions=[
                {"AttributeName": "Timestamp", "AttributeType": "N"},
            ],
        )
        tbl.wait_until_exists()
        print(f"✓ Created {IMAGE_TABLE}")


_now  = lambda: int(time.time())
_dec  = lambda n: Decimal(str(n))
table = lambda: dynamodb.Table(TABLE_NAME)
image_table = lambda: dynamodb.Table(IMAGE_TABLE)

def _build_item(
    battery: int,
    mode: str,
    action: Action,
    *,
    ts_override: int | None = None,
) -> dict[str, Any]:
    if not (0 <= battery <= 100):
        raise ValueError("battery must be 0–100")
    mode_u = mode.upper()
    if mode_u not in VALID_MODES:
        raise ValueError("mode must be AUTO or MANUAL")

    ts      = ts_override or _now()
    expires = ts + TTL_DAYS * 24 * 3600

    return {
        "RobotId": ROBOT_ID,
        "LogTS":   _dec(ts),
        "TTL":     _dec(expires),
        "Battery": _dec(battery),
        "Mode":    mode_u,
        "Action":  action.value,
    }

def _query_latest(limit: int = 1) -> List[dict]:
    resp = table().query(
        KeyConditionExpression=Key("RobotId").eq(ROBOT_ID),
        ScanIndexForward=False,
        Limit=limit,
    )
    return resp.get("Items", [])


def seed_demo_data() -> None:
    if table().scan(Select="COUNT", Limit=1)["Count"]:
        return

    print("⛵  Seeding example rows …")
    demo_rows = [
        (100, "Manual",   Action.STOP),
       
    ]
    base_ts = _now()
    with table().batch_writer() as batch:
        for off, (batt, mode, act) in enumerate(demo_rows):
            item = _build_item(batt, mode, act, ts_override=base_ts + off)
            batch.put_item(Item=item)
    print(f"✓ Inserted {len(demo_rows)} demo items")


create_table_if_missing()
create_image_table_if_missing()
seed_demo_data()  # comment out if not needed

app = FastAPI(
    title="KAY-O Operation + Image Log API",
    version="4.0.0",
    docs_url="/", redoc_url=None,
)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class LogIn(BaseModel):
    battery: int = Field(..., ge=0, le=100)
    mode:    str = Field(..., pattern=r"(?i)^(auto|manual)$")
    action:  Action

class LogOut(BaseModel):
    battery: int
    mode:    str
    action:  Action
    time:    int = Field(..., description="Epoch seconds (LogTS)")

class ImageUpload(BaseModel):
    timestamp: int
    image: str  # base64 encoded image


@app.post("/log", status_code=204)
def write_log(body: LogIn):
    table().put_item(Item=_build_item(body.battery, body.mode, body.action))
    return None

@app.get("/latest", response_model=LogOut)
def read_latest():
    items = _query_latest(1)
    if not items:
        raise HTTPException(404, "No data yet")
    it = items[0]
    return {
        "battery": int(it["Battery"]),
        "mode":    it["Mode"],
        "action":  it["Action"],
        "time":    int(it["LogTS"]),
    }

@app.get("/history", response_model=list[LogOut])
def read_history(limit: int = Query(20, ge=1, le=500)):
    items = _query_latest(limit=limit)
    return [
        {
            "battery": int(it["Battery"]),
            "mode":    it["Mode"],
            "action":  it["Action"],
            "time":    int(it["LogTS"]),
        }
        for it in items
    ]

@app.get("/image/{timestamp}", response_class=HTMLResponse)
def view_image(timestamp: int):
    key = {"Timestamp": Decimal(str(timestamp))}
    response = image_table().get_item(Key=key)
    item = response.get("Item")

    if not item or "Image" not in item:
        return HTMLResponse(
            content=f"<h2>❌ Image not found for timestamp {timestamp}</h2>",
            status_code=404,
        )

    base64_image = item["Image"]
    html = f"""
    <html>
    <head><title>Image {timestamp}</title></head>
    <body>
      <h2>🖼 Image at {timestamp}</h2>
      <img src="data:image/png;base64,{base64_image}" alt="Image" style="max-width:600px;">
    </body>
    </html>
    """
    return HTMLResponse(content=html)

@app.post("/image", status_code=204)
def upload_image(data: ImageUpload):
    image_table().put_item(Item={
        "Timestamp": _dec(data.timestamp),
        "Image": data.image
    })
    return None