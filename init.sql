-- Create trials table to store metadata about each trial
CREATE TABLE IF NOT EXISTS trial_metadata (
    id SERIAL PRIMARY KEY,
    trial_name VARCHAR(255) NOT NULL,
    description TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create simulation_data table to store input, output, and state information
CREATE TABLE IF NOT EXISTS simulation_data (
    id SERIAL PRIMARY KEY,
    trial_id INTEGER REFERENCES trial_metadata(id),
    timestep INTEGER NOT NULL,
    simulation_time DOUBLE PRECISION NOT NULL,
    input_data BYTEA,
    output_data BYTEA,
    state_data BYTEA,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);