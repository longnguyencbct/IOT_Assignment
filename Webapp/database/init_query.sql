CREATE TABLE telemetry_data (
    datetime TIMESTAMP NOT NULL, -- Column to store the date and time
    temperature FLOAT,           -- Column to store temperature readings
    humidity FLOAT,              -- Column to store humidity readings
    lux FLOAT                    -- Column to store light intensity readings
);