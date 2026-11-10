from flask import Flask, request, jsonify
from datetime import datetime

app = Flask(__name__)

# Mock storage for sensor states (could be replaced with a database)
sensor_states = {
    'Machine 1': {'State': 0},  # Example: Machine 1 is safe (1)
    'Machine 2': {'State': 0}   # Example: Machine 2 is unsafe (0)
}

@app.route('/Safety', methods=['GET'])
def handle_safety_data():
    try:
        # Check if the request contains 'Sensor' parameter
        sensor_data = request.args.get('Sensor')
        if sensor_data:
            # Handle existing Sensor data endpoint
            data_pairs = sensor_data.split(';')
            sensors = {}
            for pair in data_pairs:
                if '=' in pair:
                    key, value = pair.split('=')
                    sensors[key] = value
            
            # Check required sensor fields
            required_sensors = ['AGF_Sensor1', 'AGF_Sensor2', 'AGF_Sensor3', 'AGF_Sensor4', 'Time']
            if not all(sensor in sensors for sensor in required_sensors):
                return jsonify({"Result": "Error: Missing required sensor data"}), 400
            
            # Validate sensor values (0 or 1)
            for sensor in ['AGF_Sensor1', 'AGF_Sensor2', 'AGF_Sensor3', 'AGF_Sensor4']:
                if sensors[sensor] not in ['0', '1']:
                    return jsonify({"Result": f"Error: Invalid value for {sensor}"}), 400
            
            # Validate time format
            try:
                datetime.strptime(sensors['Time'], '%Y-%m-%d %H:%M:%S.%f')
            except ValueError:
                return jsonify({"Result": "Error: Invalid time format"}), 400
            
            # If all checks pass, return success
            return jsonify({"Result": "OK"}), 200
        
        # Check if the request contains 'Area' parameter
        area = request.args.get('Area')
        if area:
            # Handle Area query
            if area in sensor_states:
                return jsonify({"State": sensor_states[area]['State']}), 200
            else:
                return jsonify({"Result": f"Error: Unknown area {area}"}), 404
        
        # If neither Sensor nor Area is provided
        return jsonify({"Result": "Error: Missing Sensor or Area parameter"}), 400
    
    except Exception as e:
        return jsonify({"Result": f"Error: {str(e)}"}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)