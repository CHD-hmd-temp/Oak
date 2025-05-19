use bevy::ecs::system::Resource;
use serde::{Deserialize, Serialize};
use pyo3::prelude::*;

#[pyclass]
#[derive(Serialize, Deserialize, Debug, Resource, Clone)]
pub struct OakConfig {
    pub process_config: ProcessConfig,
    pub render_config: RenderConfig,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ProcessConfig {
    pub point_cloud_path: String,
    pub origin_data_path: String,
    pub sensor_angle: f32,  // X轴方向仰角，角度值
    pub mesh_vertex_threshold: f32, // 网格顶点比较阈值
    pub mesh_normal_threshold: f32, // 网格法线比较阈值
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RenderConfig {
    pub width: u32,
    pub height: u32,
    pub cam_x: f32,
    pub cam_y: f32,
    pub cam_z: f32,
    pub look_at_x: f32,
    pub look_at_y: f32,
    pub look_at_z: f32,
}

impl Default for OakConfig {
    fn default() -> Self {
        Self {
            process_config: ProcessConfig {
                point_cloud_path: "data/data-1.csv".to_string(),
                origin_data_path: "data/data-1.csv".to_string(),
                sensor_angle: 30.0,
                mesh_vertex_threshold: 0.09,
                mesh_normal_threshold: 0.05,
            },
            render_config: RenderConfig {
                width: 800,
                height: 600,
                cam_x: 0.0,
                cam_y: 4.0,
                cam_z: 4.0,
                look_at_x: 0.0,
                look_at_y: 0.0,
                look_at_z: 0.0,
            },
        }
    }
}

use std::fs::read_to_string;
pub fn load_config(config_path: &str) -> Result<OakConfig, Box<dyn std::error::Error>> {
    let mut config = OakConfig::default();

    if let Ok(config_str) = read_to_string(config_path) {
        let loaded_config: OakConfig = toml::from_str(&config_str)?;
        config = loaded_config;
    }

    Ok(config)
}