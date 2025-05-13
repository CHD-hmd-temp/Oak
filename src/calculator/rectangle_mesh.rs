#![allow(dead_code)]
use std::collections::HashMap;
use bevy::prelude::{Mesh, Vec3};
use bevy::render::mesh::Indices;
use bevy::render::render_resource::PrimitiveTopology::TriangleList;
use bevy::render::render_asset::RenderAssetUsages;
use nalgebra::Point3;
use colorous::PLASMA;
use crate::prelude::*;
use super::coordinate_transformer::mid360_to_bevy;

pub struct RectangleMesh {
    pub cell_size: f32,
    pub vertices: Vec<Vec3>,
    pub indices: Vec<u32>,
    pub normals: Vec<Vec3>,
    pub colors: Vec<[f32; 4]>,
}

impl RectangleMesh {
    pub fn from_points(points: &Vec<LaserPoint>, cell_size: f32) -> Self {
        if points.is_empty() {
            eprintln!("No points provided, returning empty mesh.");
            return Self {
                cell_size,
                vertices: Vec::new(),
                indices: Vec::new(),
                normals: Vec::new(),
                colors: Vec::new(),
            };
        }

        let mut grid: HashMap<(i32, i32), Vec<&LaserPoint>> = HashMap::new();

        let bound_min = Point3::new(0.5, -1.5, -2.0);
        let bound_max = Point3::new(2.5, 1.5, 2.0);
        let mut min_x: f32 = bound_min.x;
        let mut min_y: f32 = bound_min.y;
        let mut max_x: f32 = bound_max.x;
        let mut max_y: f32 = bound_max.y;

        for point in points {
            let x = (point.coordinate.x / cell_size).floor() as i32;
            let y = (point.coordinate.y / cell_size).floor() as i32;

            min_x = min_x.min(point.coordinate.x);
            min_y = min_y.min(point.coordinate.y);
            max_x = max_x.max(point.coordinate.x);
            max_y = max_y.max(point.coordinate.y);

            grid.entry((x, y)).or_insert_with(Vec::new).push(point);
        }

        let mut height_map: HashMap<(i32, i32), f32> = HashMap::new();

        for ((x, y), cell_points) in &grid {
            if !cell_points.is_empty() {
                let avg_z = cell_points.iter().map(|p| p.coordinate.z).sum::<f32>() / cell_points.len() as f32;
                height_map.insert((*x, *y), avg_z);
            }
        }

        let x_min = ((min_x / cell_size).floor() as i32).clamp(-1000, 1000);
        let y_min = ((min_y / cell_size).floor() as i32).clamp(-1000, 1000);
        let x_max = ((max_x / cell_size).ceil() as i32).clamp(-1000, 1000);
        let y_max = ((max_y / cell_size).ceil() as i32).clamp(-1000, 1000);

        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut normals = Vec::new();
        let mut colors = Vec::new();

        let mut vertex_map: HashMap<(i32, i32), u32> = HashMap::new();

        for y in y_min..=y_max {
            for x in x_min..=x_max {
                let z_opt = if let Some(z) = height_map.get(&(x, y)) {
                    Some(*z)
                } else {
                    let neighbours = [
                        height_map.get(&(x - 1, y)),
                        height_map.get(&(x + 1, y)),
                        height_map.get(&(x, y - 1)),
                        height_map.get(&(x, y + 1)),
                    ];
                    let has_neighbour = neighbours.iter().any(|&n| n.is_some());
                    if has_neighbour {
                        let sum: f32 = neighbours.iter().filter_map(|&n| n.copied()).sum();
                        let count = neighbours.iter().filter(|&&n| n.is_some()).count() as f32;
                        Some(sum / count)
                    } else {
                        None
                    }
                };

                if let Some(z) = z_opt {
                    let position = Vec3::new(x as f32 * cell_size, y as f32 * cell_size, z);
                    vertex_map.insert((x, y), vertices.len() as u32);
                    vertices.push(position);
                    normals.push(Vec3::ZERO);
                }
            }
        }

        for y in y_min..y_max {
            for x in x_min..x_max {
                if let (Some(&tl), Some(&tr), Some(&bl), Some(&br)) = (
                    vertex_map.get(&(x, y + 1)),
                    vertex_map.get(&(x + 1, y + 1)),
                    vertex_map.get(&(x, y)),
                    vertex_map.get(&(x + 1, y)),
                ) {
                    indices.push(tl);
                    indices.push(bl);
                    indices.push(br);

                    indices.push(tl);
                    indices.push(br);
                    indices.push(tr);
                }
            }
        }

        Self::calculate_normals(&mut vertices, &indices, &mut normals);

        for normal in &normals {
            let slope_rad = normal.z.clamp(-1.0, 1.0).acos();
            let col = PLASMA.eval_continuous((slope_rad as f64).clamp(0.0, 1.0));
            let color: [f32; 4] = [
                col.r as f32 / 255.0,         // Red
                col.g as f32 / 255.0,         // Green
                col.b as f32 / 255.0,         // Blue
                1.0,                  // Alpha
            ];
            colors.push(color);
        }

        RectangleMesh {
            cell_size,
            vertices,
            indices,
            normals,
            colors,
        }
    }

    fn calculate_normals(vertices: &[Vec3], indices: &[u32], normals: &mut [Vec3]) {
        for normal in normals.iter_mut() {
            *normal = Vec3::ZERO;
        }

        for i in (0..indices.len()).step_by(3) {
            let v0 = vertices[indices[i] as usize];
            let v1 = vertices[indices[i + 1] as usize];
            let v2 = vertices[indices[i + 2] as usize];

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let face_normal = edge1.cross(edge2).normalize();

            normals[indices[i] as usize] += face_normal;
            normals[indices[i + 1] as usize] += face_normal;
            normals[indices[i + 2] as usize] += face_normal;
        }

        for normal in normals.iter_mut() {
            if normal.length_squared() > 0.0 {
                *normal = normal.normalize();
            } else {
                *normal = Vec3::new(0.0, 0.0, 1.0);
            }
        }
    }

    pub fn vertices(&self) -> &Vec<Vec3> {
        &self.vertices
    }

    pub fn indices(&self) -> &Vec<u32> {
        &self.indices
    }

    pub fn normals(&self) -> &Vec<Vec3> {
        &self.normals
    }

    pub fn to_bevy_mesh(&self) -> Mesh {
        let mut mesh = Mesh::new(TriangleList, RenderAssetUsages::RENDER_WORLD);
    
        // Transform the vertices to Bevy coordinates and convert to Bevy's Vec3
        let transformed_vertices: Vec<Vec3> = self.vertices.iter()
            .map(|v| {
                let converted = mid360_to_bevy(v.x, v.y, v.z);
                Vec3::new(converted.x, converted.y, converted.z)
            })
            .collect();
    
        // Transform the normals to Bevy coordinates and convert to Bevy's Vec3
        let transformed_normals: Vec<Vec3> = self.normals.iter()
            .map(|n| {
                let converted = mid360_to_bevy(n.x, n.y, n.z);
                Vec3::new(converted.x, converted.y, converted.z)
            })
            .collect();

        // Insert the transformed vertices, normals, and colors into the mesh
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, transformed_vertices);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, transformed_normals);
        mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, self.colors.clone());
        mesh.insert_indices(Indices::U32(self.indices.clone()));
    
        mesh
    }
}
