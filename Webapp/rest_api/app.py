from flask import Flask, request, jsonify
from flask_cors import CORS
import psycopg2
from datetime import datetime

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

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

@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    try:
        conn = get_db_connection()
        cursor = conn.cursor()

        # Fetch telemetry data
        cursor.execute("SELECT * FROM telemetry_data ORDER BY datetime DESC")
        rows = cursor.fetchall()

        cursor.close()
        conn.close()

        # Convert rows to a list of dictionaries
        data = [
            {"datetime": row[0], "temperature": row[1], "humidity": row[2], "lux": row[3]}
            for row in rows
        ]

        return jsonify(data), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)