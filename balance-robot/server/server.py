# server.py  –  FastAPI + DynamoDB log (KAY-O)  ▓▓▓  WITH DEMO DATA  ▓▓▓
# ──────────────────────────────────────────────────────────────────────────
# Endpoints
#   POST /log         -> write one row  (battery, mode, action)
#   GET  /latest      -> newest row
#   GET  /history     -> last N rows    (default 20)
#
# The table is created automatically and, if currently empty, seeded with
# five example rows so you can see something immediately in Swagger UI or
# the AWS console.
#
# Run:
#   uvicorn server:app --host 0.0.0.0 --port 8001 --reload
# --------------------------------------------------------------------------

from __future__ import annotations

import os, time
from decimal import Decimal
from enum import Enum
from typing import Any, List

import boto3
from boto3.dynamodb.conditions import Key
from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

# ────────────────── CONFIG ──────────────────
ROBOT_ID   = "KAY-O"                             # <- fixed robot name
TABLE_NAME = os.getenv("ROBOTLOG_TABLE", "RobotLogs")
REGION     = os.getenv("AWS_REGION", "eu-west-2")
TTL_DAYS   = int(os.getenv("TTL_DAYS", 30))
ENDPOINT   = os.getenv("DDB_ENDPOINT_URL")       # set for DynamoDB Local

ddb_kwargs: dict[str, Any] = {"region_name": REGION}
if ENDPOINT:
    ddb_kwargs["endpoint_url"] = ENDPOINT
dynamodb = boto3.resource("dynamodb", **ddb_kwargs)
client   = boto3.client("dynamodb",  **ddb_kwargs)

# ───────────── ENUM & CONSTANTS ─────────────
class Action(str, Enum):
    FORWARD  = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT     = "LEFT"
    RIGHT    = "RIGHT"
    STOP     = "STOP"

VALID_MODES = {"AUTO", "MANUAL"}

# ──────────── TABLE MANAGEMENT ──────────────
def create_table_if_missing() -> None:
    """Create RobotLogs (PK: RobotId, SK: LogTS) & turn on TTL."""
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

# ───────────── LOW-LEVEL HELPERS ─────────────
_now  = lambda: int(time.time())
_dec  = lambda n: Decimal(str(n))
table = lambda: dynamodb.Table(TABLE_NAME)

def _build_item(
    battery: int,
    mode: str,
    action: Action,
    *,
    ts_override: int | None = None,      # used only by seeder
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

# ─────────────── DEMO SEEDER ────────────────
def seed_demo_data() -> None:
    """
    Insert five sample rows if the table is empty.
    Ensures unique (RobotId, LogTS) by adding +1 s offset per row.
    """
    if table().scan(Select="COUNT", Limit=1)["Count"]:
        return                              # already data → skip seeding

    print("⛵  Seeding example rows …")
    demo_rows = [
        (95, "AUTO",   Action.FORWARD),
        (92, "AUTO",   Action.RIGHT),
        (88, "AUTO",   Action.RIGHT),
        (70, "MANUAL", Action.LEFT),
        (65, "MANUAL", Action.STOP),
    ]
    base_ts = _now()
    with table().batch_writer() as batch:
        for off, (batt, mode, act) in enumerate(demo_rows):
            item = _build_item(batt, mode, act, ts_override=base_ts + off)
            batch.put_item(Item=item)
    print(f"✓ Inserted {len(demo_rows)} demo items")

# ───────────────── FASTAPI ───────────────────
create_table_if_missing()
seed_demo_data()           # <-- remove this line if you don’t want seeding

app = FastAPI(
    title="KAY-O Operation Log API",
    version="3.1.0",
    docs_url="/", redoc_url=None,
)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ───────────── DTOs (request / response) ─────────────
class LogIn(BaseModel):
    battery: int = Field(..., ge=0, le=100, description="Battery %")
    mode:    str = Field(
        ...,
        pattern=r"(?i)^(auto|manual)$",
        description="'AUTO' or 'MANUAL'",
    )
    action:  Action

class LogOut(BaseModel):
    battery: int
    mode:    str
    action:  Action
    time:    int = Field(..., description="Epoch seconds (LogTS)")

# ────────────────── ROUTES ───────────────────
@app.post("/log", status_code=204)
def write_log(body: LogIn):
    """Write one operation row for KAY-O."""
    table().put_item(Item=_build_item(body.battery, body.mode, body.action))
    return None                               # 204 No Content

@app.get("/latest", response_model=LogOut)
def read_latest():
    """Return the newest stored row for KAY-O."""
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
    """Return *limit* most-recent rows (newest → oldest)."""
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
