import json
import boto3
import datetime
import base64
from datetime import datetime, timedelta, timezone

s3 = boto3.client("s3")

BUCKET_NAME = "fromesptos3128959305651"
FILE_KEY = "all_devices/data.json" 


def lambda_handler(event, context):
    try:
        # ---------------------------------------
        # 1. Read JSON from event
        # ---------------------------------------
        if "body" in event:
            if event.get("isBase64Encoded", False):
                body = base64.b64decode(event["body"])
                data = json.loads(body)
            else:
                data = json.loads(event["body"])
        else:
            data = event

        # ---------------------------------------
        # Add timestamp with UTC+4
        # ---------------------------------------
        tz_plus_4 = timezone(timedelta(hours=4))
        data["timestamp"] = datetime.now(tz_plus_4).isoformat()

        # ---------------------------------------
        # 2. Load existing file from S3 (if exists)
        # ---------------------------------------
        try:
            response = s3.get_object(Bucket=BUCKET_NAME, Key=FILE_KEY)
            file_content = response["Body"].read().decode("utf-8")

            # existing json array
            json_array = json.loads(file_content)

            if not isinstance(json_array, list):
                json_array = [json_array]

        except s3.exceptions.NoSuchKey:
            # file does not exist yet → create new list
            json_array = []

        # ---------------------------------------
        # 3. Append new record
        # ---------------------------------------
        json_array.append(data)

        # ---------------------------------------
        # 4. Save the updated array back into S3
        # ---------------------------------------
        s3.put_object(
            Bucket=BUCKET_NAME,
            Key=FILE_KEY,
            Body=json.dumps(json_array, indent=2),
            ContentType="application/json"
        )

        return {
            "statusCode": 200,
            "body": json.dumps({
                "message": "Data appended to single JSON file",
                "records_total": len(json_array)
            })
        }

    except Exception as e:
        print("Error:", str(e))
        return {
            "statusCode": 500,
            "body": json.dumps({"error": str(e)})
        }