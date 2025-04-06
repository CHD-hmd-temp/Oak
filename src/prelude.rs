#![allow(dead_code)]
// This file contains the prelude for the project, which includes commonly used modules and types.

#[derive(Clone, Copy)]
pub struct LaserPoint {
    pub coordinate: nalgebra::Point3<f32>,
    pub reflectivity: u8,
}

impl LaserPoint {
    pub fn new(point: nalgebra::Point3<f32>, reflectivity: u8) -> Self {
        Self {
            coordinate: point,
            reflectivity,
        }
    }
}