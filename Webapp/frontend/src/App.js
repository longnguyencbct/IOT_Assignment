import logo from './logo.svg';
import React, { useState, useEffect } from 'react'; // Add useState and useEffect here
import './App.css';

function App() {
  const [telemetryData, setTelemetryData] = useState([]);

  useEffect(() => {
    const apiUrl ='http://localhost:5000';
    fetch(`${apiUrl}/api/telemetry`)
      .then((response) => response.json())
      .then((data) => setTelemetryData(data))
      .catch((error) => console.error('Error fetching telemetry data:', error));
  }, []);

  return (
    <div className="App">
      <header className="App-header">
        <h1>Telemetry Data</h1>
        <ul>
          {telemetryData.map((entry, index) => (
            <li key={index}>
              {entry.datetime}: Temp={entry.temperature}°C, Humidity={entry.humidity}%, Lux={entry.lux}
            </li>
          ))}
        </ul>
      </header>
    </div>
  );
}

export default App;
