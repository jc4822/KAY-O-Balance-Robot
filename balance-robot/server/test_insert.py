from __future__ import annotations
import os
import time
from decimal import Decimal
from enum import Enum
import boto3

# ───────────── ENUM & CONSTANTS ─────────────
class Action(str, Enum):
    FORWARD  = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT     = "LEFT"
    RIGHT    = "RIGHT"
    STOP     = "STOP"

VALID_MODES = {"AUTO", "MANUAL"}
DEFAULT_ROBOT_ID = os.getenv("ROBOT_ID", "KAY-O")
TABLE_NAME      = os.getenv("ROBOTLOG_TABLE", "RobotLogs")
REGION          = os.getenv("AWS_REGION", "eu-west-2")
TTL_DAYS        = int(os.getenv("TTL_DAYS", 30))
DDB_ENDPOINT    = os.getenv("DDB_ENDPOINT_URL")  # for local testing

# ─────────────── HELPER BUILDERS ───────────────
_now = lambda: int(time.time())

def build_item(
    battery: int,
    mode: str,
    action: Action,
    *,
    ts_override: int | None = None,
) -> dict[str, any]:
    """
    Build the DynamoDB item payload with TTL attribute.
    """
    if not (0 <= battery <= 100):
        raise ValueError("battery must be between 0 and 100")
    mode_u = mode.upper()
    if mode_u not in VALID_MODES:
        raise ValueError("mode must be 'AUTO' or 'MANUAL'")

    ts = ts_override or _now()
    expires = ts + TTL_DAYS * 24 * 3600

    return {
        "RobotId": DEFAULT_ROBOT_ID,
        "LogTS":   Decimal(str(ts)),
        "TTL":     Decimal(str(expires)),
        "Battery": Decimal(str(battery)),
        "Mode":    mode_u,
        "Action":  action.value,
    }

# ─────────────── MAIN SCRIPT ───────────────
def main():
    # Initialize DynamoDB resource
    kwargs: dict[str, any] = {"region_name": REGION}
    if DDB_ENDPOINT:
        kwargs["endpoint_url"] = DDB_ENDPOINT

    dynamodb = boto3.resource("dynamodb", **kwargs)
    table = dynamodb.Table(TABLE_NAME)

    # Define your batch of data here
    to_insert = [
        {"battery": 95, "mode": "AUTO",   "action": Action.FORWARD},
        {"battery": 90, "mode": "AUTO",   "action": Action.RIGHT},
        {"battery": 85, "mode": "AUTO",   "action": Action.RIGHT},
        {"battery": 80, "mode": "MANUAL", "action": Action.LEFT},
        {"battery": 75, "mode": "MANUAL", "action": Action.STOP},
        # add more entries as needed
    ]

    # Ensure unique LogTS by offsetting each timestamp
    base_ts = _now()
    with table.batch_writer() as batch:
        for idx, entry in enumerate(to_insert):
            ts = base_ts + idx
            item = build_item(
                entry["battery"], entry["mode"], entry["action"],
                ts_override=ts
            )
            batch.put_item(Item=item)

    print(f"Inserted {len(to_insert)} items into '{TABLE_NAME}' with unique timestamps")


if __name__ == "__main__":
    main()
