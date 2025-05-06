use std::fs::File;
use std::io::{self, BufRead};
use std::path::Path;
use crate::prelude::LaserPoint;
use crate::calculator::coordinate_transformer::{mid360_to_bevy, transform_to_normal_position};
use nalgebra::Point3;

// Read PointCloud from a csv file
pub fn read_point_cloud_from_csv<P: AsRef<Path>>(file_path: P) -> io::Result<Vec<LaserPoint>> {
    let file = File::open(file_path)?;
    let reader = io::BufReader::new(file);
    let mut points = Vec::new();

    let bound_min = Point3::new(0.9, -2.0, -2.0);
    let bound_max = Point3::new(2.5, 2.0, 2.0);
    
    // Skip the header line
    for line in reader.lines().skip(2) {
        let line = line?;
        let fields: Vec<&str> = line.split(',').collect();
        
        // Make sure we have enough fields
        if fields.len() < 12 {
            continue;
        }
        
        // Parse X, Y, Z and reflectivity
        if let (Ok(x), Ok(y), Ok(z), Ok(reflectivity)) = (
            fields[8].parse::<f32>(),
            fields[9].parse::<f32>(),
            fields[10].parse::<f32>(),
            fields[11].parse::<u8>(),
        ) {
            let point = Point3::new(x, y, z);
            
            // Check if the point is within the bounds
            if point.x >= bound_min.x && point.x <= bound_max.x &&
               point.y >= bound_min.y && point.y <= bound_max.y &&
               point.z >= bound_min.z && point.z <= bound_max.z {
                let point = transform_to_normal_position(x, y, z);
                //let point = mid360_to_bevy(point.x, point.y, point.z);
                points.push(LaserPoint::new(point, reflectivity));
            }
        } else {
            eprintln!("Failed to parse line: {}", line);
        }
    }
    
    Ok(points)
}