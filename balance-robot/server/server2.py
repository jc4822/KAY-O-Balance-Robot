"""
server.py – Robot log helper library PLUS a FastAPI service
Run with:
    uvicorn server:app --host 0.0.0.0 --port 8001   # or any port you like
Environment you still need:
    AWS_REGION       (defaults to eu-west-2 – London)
    ROBOTLOG_TABLE   (defaults to RobotLogs)
    TTL_DAYS         (defaults to 30)
"""
from __future__ import annotations
import os, time, json
from decimal import Decimal
from enum import Enum
from typing import Any, Optional

import boto3
from botocore.exceptions import ClientError
from boto3.dynamodb.conditions import Key

# ─────────────────────────── DynamoDB CONFIG ────────────────────────────
TABLE_NAME = os.getenv("ROBOTLOG_TABLE", "RobotLogs")
REGION     = os.getenv("AWS_REGION",     "eu-west-2")
TTL_DAYS   = int(os.getenv("TTL_DAYS",   30))

dynamodb = boto3.resource("dynamodb", region_name=REGION)
client   = boto3.client("dynamodb",   region_name=REGION)

# ──────────────────────────────── ENUMS ──────────────────────────────────
class Motion(str, Enum):
    FORWARD  = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT     = "LEFT"
    RIGHT    = "RIGHT"
    STOP     = "STOP"

VALID_MODES = {"AUTO", "MANUAL"}

# ─────────────────────── TABLE CREATION (idempotent) ────────────────────
def create_table_if_missing() -> None:
    """Create the RobotLogs table if it does not yet exist."""
    try:
        dynamodb.Table(TABLE_NAME).load()
        return                                               # Already exists
    except client.exceptions.ResourceNotFoundException:
        tbl = dynamodb.create_table(
            TableName            = TABLE_NAME,
            BillingMode          = "PAY_PER_REQUEST",
            KeySchema            = [
                {"AttributeName": "RobotId", "KeyType": "HASH"},
                {"AttributeName": "LogTS",  "KeyType": "RANGE"},
            ],
            AttributeDefinitions = [
                {"AttributeName": "RobotId", "AttributeType": "S"},
                {"AttributeName": "LogTS",   "AttributeType": "N"},
            ],
        )
        tbl.wait_until_exists()
        client.update_time_to_live(
            TableName = TABLE_NAME,
            TimeToLiveSpecification={"Enabled": True, "AttributeName": "TTL"},
        )
        print(f"✓ Created {TABLE_NAME} (TTL enabled)")

# ────────────────────────────── HELPERS ──────────────────────────────────
def _epoch() -> int:
    return int(time.time())

def _dec(n: int | float) -> Decimal:
    return Decimal(str(n))

def _put(item: dict) -> None:
    dynamodb.Table(TABLE_NAME).put_item(Item=item)

def _make_item(
    robot_id: str,
    battery: Optional[int],
    motion:  Optional[Motion],
    mode:    Optional[str],
    message: str,
    ttl_days: int,
) -> dict[str, Any]:
    ts      = _epoch()
    expires = ts + ttl_days * 24 * 3600

    if battery is not None and not (0 <= battery <= 100):
        raise ValueError("battery must be 0-100")
    if motion is not None and motion not in Motion:
        raise ValueError(f"motion must be one of {list(Motion)}")
    if mode is not None and mode.upper() not in VALID_MODES:
        raise ValueError("mode must be AUTO or MANUAL")

    item: dict[str, Any] = {
        "RobotId": robot_id,
        "LogTS":   _dec(ts),
        "TTL":     _dec(expires),
        "Message": message,
    }
    if battery is not None:
        item["Battery"] = _dec(battery)
    if motion is not None:
        item["Motion"] = motion.value
    if mode is not None:
        item["Mode"] = mode.upper()
    return item

def _latest_item(robot_id: str, projection: str | None = None,
                 attr_names: dict[str, str] | None = None) -> dict | None:
    table = dynamodb.Table(TABLE_NAME)
    kwargs: dict = {
        "KeyConditionExpression": Key("RobotId").eq(robot_id),
        "ScanIndexForward": False,     # newest first
        "Limit": 1,
    }
    if projection:
        kwargs["ProjectionExpression"] = projection
    if attr_names:
        kwargs["ExpressionAttributeNames"] = attr_names
    resp = table.query(**kwargs)
    items = resp.get("Items", [])
    return items[0] if items else None

# ───────────────────────────── PUBLIC API ────────────────────────────────
def update_status(
    robot_id: str,
    *,
    battery: Optional[int] = None,
    motion:  Optional[Motion] = None,
    mode:    Optional[str] = None,
    message: str = "",
    ttl_days: int = TTL_DAYS,
) -> None:
    if battery is None and motion is None and mode is None:
        raise ValueError("Must set at least one of battery, motion, or mode")
    _put(_make_item(robot_id, battery, motion, mode, message, ttl_days))

def get_battery(robot_id: str) -> Optional[int]:
    item = _latest_item(robot_id, projection="Battery")
    return int(item["Battery"]) if item and "Battery" in item else None

def get_mode(robot_id: str) -> Optional[str]:
    item = _latest_item(robot_id,
                        projection="#m",
                        attr_names={"#m": "Mode"})
    return item["Mode"] if item and "Mode" in item else None

def get_full_status(robot_id: str) -> dict | None:
    item = _latest_item(robot_id, projection="Battery,Mode,Motion,Message")
    if not item:
        return None
    return {
        "battery": int(item["Battery"]) if "Battery" in item else None,
        "mode":     item.get("Mode"),
        "motion":   item.get("Motion"),
        "message":  item.get("Message", ""),
        "timestamp": int(item["LogTS"]),
    }

# ─────────────────────────── FASTAPI SERVICE ─────────────────────────────
create_table_if_missing()

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(
    title="Robot Status API",
    version="1.0",
    docs_url="/",
    redoc_url=None
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],           # dev-friendly; lock down in prod
    allow_methods=["*"],
    allow_headers=["*"],
)

# DTOs -------------------------------------------------------------------
class StatusIn(BaseModel):
    battery: Optional[int]  = None
    motion:  Optional[Motion] = None
    mode:    Optional[str]  = None
    message: str            = ""

# Routes -----------------------------------------------------------------
@app.get("/robots/{robot_id}/status")
def read_status(robot_id: str):
    data = get_full_status(robot_id)
    if not data:
        raise HTTPException(404, "No status for robot yet")
    return data

@app.post("/robots/{robot_id}/status", status_code=204)
def write_status(robot_id: str, body: StatusIn):
    update_status(robot_id,
                  battery=body.battery,
                  motion=body.motion,
                  mode=body.mode,
                  message=body.message)
    return None            # 204 No Content

# Convenience single-field getters (optional) ----------------------------
@app.get("/robots/{robot_id}/battery")
def read_battery(robot_id: str):
    b = get_battery(robot_id)
    if b is None:
        raise HTTPException(404, "Unknown")
    return {"battery": b}

@app.get("/robots/{robot_id}/mode")
def read_mode(robot_id: str):
    m = get_mode(robot_id)
    if m is None:
        raise HTTPException(404, "Unknown")
    return {"mode": m}
