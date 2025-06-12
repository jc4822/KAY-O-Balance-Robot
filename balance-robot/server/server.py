from __future__ import annotations
import os, time, json
from decimal import Decimal
from enum import Enum
import boto3
from botocore.exceptions import ClientError
from boto3.dynamodb.conditions import Key

# ─────────────────────────── configurable constants ────────────────────────────
TABLE_NAME = os.getenv("ROBOTLOG_TABLE", "RobotLogs")
REGION     = os.getenv("AWS_REGION",     "eu-west-2")   # London
TTL_DAYS   = int(os.getenv("TTL_DAYS",   30))           # item expiry

dynamodb = boto3.resource("dynamodb", region_name=REGION)
client   = boto3.client("dynamodb",   region_name=REGION)

# ──────────────────────────────── ENUMS & CONSTANTS ────────────────────────────
class Motion(str, Enum):
    FORWARD  = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT     = "LEFT"
    RIGHT    = "RIGHT"
    STOP     = "STOP"

VALID_MODES = {"AUTO", "MANUAL"}

# ──────────────────────── TABLE CREATION / BOOTSTRAP ───────────────────────────
def create_table_if_missing() -> None:
    """Create the RobotLogs table if absent (PK: RobotId, SK: LogTS)."""
    try:
        dynamodb.Table(TABLE_NAME).load()      # already exists
        return
    except client.exceptions.ResourceNotFoundException:
        tbl = dynamodb.create_table(
            TableName=TABLE_NAME,
            BillingMode="PAY_PER_REQUEST",
            KeySchema=[
                {"AttributeName": "RobotId", "KeyType": "HASH"},   # PK
                {"AttributeName": "LogTS",  "KeyType": "RANGE"},   # SK
            ],
            AttributeDefinitions=[
                {"AttributeName": "RobotId", "AttributeType": "S"},
                {"AttributeName": "LogTS",  "AttributeType": "N"},
            ],
        )
        tbl.wait_until_exists()
        client.update_time_to_live(
            TableName=TABLE_NAME,
            TimeToLiveSpecification={"Enabled": True, "AttributeName": "TTL"},
        )
        print(f"✓ Created {TABLE_NAME} with TTL enabled.")

# ──────────────────────────────── INTERNAL HELPERS ─────────────────────────────
def _epoch() -> int:
    return int(time.time())

def _decimal(n: int | float) -> Decimal:
    return Decimal(str(n))

def _put_item(item: dict) -> None:
    dynamodb.Table(TABLE_NAME).put_item(Item=item)

def _make_item(
    robot_id: str,
    battery: int | None,
    motion: Motion | None,
    mode: str | None,
    message: str,
    ttl_days: int,
) -> dict:
    """Build the attribute map with validation."""
    ts      = _epoch()
    expires = ts + ttl_days * 24 * 3600

    if battery is not None and not (0 <= battery <= 100):
        raise ValueError("battery must be 0–100")
    if motion is not None and motion not in Motion:
        raise ValueError(f"motion must be one of {list(Motion)}")
    if mode is not None and mode.upper() not in VALID_MODES:
        raise ValueError("mode must be 'AUTO' or 'MANUAL'")

    item: dict = {
        "RobotId": robot_id,
        "LogTS":   _decimal(ts),
        "TTL":     _decimal(expires),
        "Message": message,
    }
    if battery is not None:
        item["Battery"] = _decimal(battery)
    if motion is not None:
        item["Motion"] = motion.value
    if mode is not None:
        item["Mode"] = mode.upper()
    return item

def _latest_item(
    robot_id: str,
    *,
    projection: str | None = None,
    expr_attr_names: dict[str, str] | None = None,
) -> dict | None:
    """
    Return the newest item for a robot.

    Parameters
    ----------
    robot_id : str
        The robot we’re querying.
    projection : str | None
        A comma-separated ProjectionExpression (e.g. "Battery,Mode").
    expr_attr_names : dict[str, str] | None
        ExpressionAttributeNames mapping for any reserved words used
        in the ProjectionExpression.
    """
    table = dynamodb.Table(TABLE_NAME)

    kwargs: dict = {
        "KeyConditionExpression": Key("RobotId").eq(robot_id),
        "ScanIndexForward": False,   # newest first
        "Limit": 1,
    }
    if projection:
        kwargs["ProjectionExpression"] = projection
    if expr_attr_names:
        kwargs["ExpressionAttributeNames"] = expr_attr_names

    resp  = table.query(**kwargs)
    items = resp.get("Items", [])
    return items[0] if items else None

# ─────────────────────────────── PUBLIC API ────────────────────────────────────
def update_status(
    robot_id: str,
    *,
    battery: int | None = None,
    motion:  Motion | None = None,
    mode:    str | None = None,
    message: str = "",
    ttl_days: int = TTL_DAYS,
) -> None:
    """
    Write *one* log line that may include updated battery, motion, and/or mode.
    Supply only the fields that actually changed.
    """
    if battery is None and motion is None and mode is None:
        raise ValueError("At least one of battery, motion or mode must be set")
    item = _make_item(robot_id, battery, motion, mode, message, ttl_days)
    _put_item(item)

# Convenience wrappers ---------------------------------------------------------
def set_battery(robot_id: str, battery: int, *, message: str = "") -> None:
    """Log a new battery level."""
    update_status(robot_id, battery=battery, message=message)

def set_mode(robot_id: str, mode: str, *, message: str = "") -> None:
    """Log a mode change ('AUTO' | 'MANUAL')."""
    update_status(robot_id, mode=mode, message=message)

# Getters ----------------------------------------------------------------------
def get_battery(robot_id: str) -> int | None:
    """Return the latest battery % for the robot, or None if unknown."""
    item = _latest_item(robot_id, projection="Battery")
    return int(item["Battery"]) if item and "Battery" in item else None

def get_mode(robot_id: str) -> str | None:
    """Return 'AUTO' or 'MANUAL' (latest), or None if unknown."""
    item = _latest_item(
        robot_id,
        projection="#m",                 # alias in ProjectionExpression
        expr_attr_names={"#m": "Mode"},  # reserved word mapping
    )
    return item["Mode"] if item and "Mode" in item else None

# ────────────────────────────── DEMO / SELF-TEST ───────────────────────────────
if __name__ == "__main__":
    create_table_if_missing()

    # Example usage -------------------------------------------------------------
    update_status(
        robot_id="R2-D2",
        battery=85,
        motion=Motion.FORWARD,
        mode="AUTO",
        message="Patrolling sector-7",
    )
    set_battery("R2-D2", 84, message="Battery drain observed")
    set_mode("R2-D2", "MANUAL", message="Operator takeover")

    print("Latest battery:", get_battery("R2-D2"))
    print("Latest mode   :", get_mode("R2-D2"))
