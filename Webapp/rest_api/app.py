from flask import Flask, request, jsonify
import psycopg2
from datetime import datetime

app = Flask(__name__)

# Database connection settings
DB_HOST = "database"
DB_NAME = "iot_database"
DB_USER = "admin"
DB_PASSWORD = "admin"

def get_db_connection():
    return psycopg2.connect(
        host=DB_HOST,
        database=DB_NAME,
        user=DB_USER,
        password=DB_PASSWORD
    )

@app.route('/api/telemetry', methods=['POST'])
def save_telemetry():
    data = request.json
    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        # Insert telemetry data into the database
        cursor.execute(
            """
            INSERT INTO telemetry_data (datetime, temperature, humidity, lux)
            VALUES (%s, %s, %s, %s)
            """,
            (
                datetime.now(),
                data.get("temperature"),
                data.get("humidity"),
                data.get("lux")
            )
        )
        conn.commit()
        cursor.close()
        conn.close()

        return jsonify({"message": "Telemetry data saved successfully"}), 201
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)