mod prelude;
mod data_reader;
mod calculator;
mod interface;
mod config;

fn main() {
    // Load the configuration
    let config_path = "config.toml";
    let oak_config = match config::load_config(config_path) {
        Ok(config) => config,
        Err(e) => {
            eprintln!("Error loading config: {}", e);
            config::OakConfig::default()
        }
    };

    // Run the Bevy app
    interface::rendering_component::run_bevy(oak_config.clone());
}

#[allow(dead_code)]
fn icp() {
    let oak_config = config::OakConfig::default();
    let target_file_path = "data/data-1-3.csv";
    let target = match data_reader::read_from_file::read_point_cloud_from_csv(target_file_path, &oak_config) {
        Ok(points) => {
            points
        }
        Err(e) => {
            eprintln!("Error reading point cloud: {}", e);
            return;
        }
    };

    let source_file_path = "data/data-1.csv";
    let source = match data_reader::read_from_file::read_point_cloud_from_csv(source_file_path, &oak_config) {
        Ok(points) => {
            points
        }
        Err(e) => {
            eprintln!("Error reading point cloud: {}", e);
            return;
        }
    };

    // Example usage of the ICP function
    
    let iterations = 10;
    let (rotation, translation) = calculator::icp::icp(&source, &target, iterations);
    println!("Rotation: {:?}", rotation);
    println!("Translation: {:?}", translation);
}
