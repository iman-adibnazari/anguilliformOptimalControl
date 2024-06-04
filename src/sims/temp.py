import numpy as np
import psycopg2
import pickle
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_db_connection():
    conn = psycopg2.connect(
        dbname='simDB',
        user='user',
        password='password',
        host='localhost',
        port='5432'
    )
    return conn

def fetch_trial_data(conn, trial_id):
    try:
        cur = conn.cursor()
        
        # Fetch data for the given trial_id
        cur.execute('SELECT timestep, simulation_time, input_data, output_data, state_data FROM simulation_data WHERE trial_id = %s', (trial_id,))
        rows = cur.fetchall()
        
        # Deserialize data
        data = []
        for row in rows:
            timestep, simulation_time, input_data_bin, output_data_bin, state_data_bin = row
            input_data = pickle.loads(input_data_bin)
            output_data = pickle.loads(output_data_bin)
            state_data = pickle.loads(state_data_bin)
            data.append((timestep, simulation_time, input_data, output_data, state_data))
        
        cur.close()
        return data
    except Exception as e:
        logger.error(f"Error fetching data for trial_id {trial_id}: {e}")
        return None

# Example usage
if __name__ == "__main__":
    conn = get_db_connection()
    
    trial_id = 5  # Replace with the trial_id you want to fetch
    trial_data = fetch_trial_data(conn, trial_id)
    
    if trial_data:
        for timestep, simulation_time, input_data, output_data, state_data in trial_data:
            print(f"Timestep: {timestep}, Simulation Time: {simulation_time}")
            print(f"Input Data: {input_data}")
            print(f"Output Data: {output_data}")
            print(f"State Data: {state_data}")
    
    conn.close()
