#![allow(dead_code)]
use crate::prelude::*;
use nalgebra::{Vector3, Matrix3, SVD};

fn compute_best_fit_transform(source: &[Vector3<f32>], target: &[Vector3<f32>]) -> (Matrix3<f32>, Vector3<f32>) {
    // Compute centroids of source and target
    let centroid_source = source.iter().sum::<Vector3<f32>>() / (source.len() as f32);
    let centroid_target = target.iter().sum::<Vector3<f32>>() / (target.len() as f32);
    
    // Remove centroids from points (demean)
    let source_demean: Vec<_> = source.iter().map(|p| p - centroid_source).collect();
    let target_demean: Vec<_> = target.iter().map(|p| p - centroid_target).collect();
    
    // Compute cross-covariance matrix
    let mut covariance = Matrix3::zeros();
    for (s, t) in source_demean.iter().zip(target_demean.iter()) {
        covariance += t * s.transpose();
    }
    
    // Compute SVD
    let svd = SVD::new(covariance, true, true);
    let u = svd.u.unwrap();
    let v_t = svd.v_t.unwrap();
    
    let mut rotation = u * v_t;
    // Handle possible reflection issue
    if rotation.determinant() < 0.0 {
        let mut u_corrected = u.clone();
        u_corrected.column_mut(2).iter_mut().for_each(|x| *x *= -1.0);
        rotation = u_corrected * v_t;
    }
    
    // Compute translation
    let translation = centroid_target - rotation * centroid_source;
    
    (rotation, translation)
}

// In your main ICP loop:
pub fn icp(source_points: &[LaserPoint], target_points: &[LaserPoint], iterations: i32) -> (Matrix3<f32>, Vector3<f32>) {
    // Convert LaserPoint to nalgebra::Vector3<f32>
    let source: Vec<_> = source_points.iter()
        .map(|p| Vector3::new(p.coordinate.x, p.coordinate.y, p.coordinate.z))
        .collect();
    let target: Vec<_> = target_points.iter()
        .map(|p| Vector3::new(p.coordinate.x, p.coordinate.y, p.coordinate.z))
        .collect();
    
    let mut r_total = Matrix3::identity();
    let mut t_total = Vector3::zeros();
    
    for _ in 0..iterations {
        let transformed_source: Vec<_> = source.iter()
            .map(|p| r_total * p + t_total)
            .collect();

        // Find closest points in target for each point in transformed_source
        let correspondences = find_correspondences(&transformed_source, &target);
        // Check if correspondences are valid (e.g., not empty)
        if correspondences.is_empty() {
            break; // No correspondences found, exit loop
        }

        // Compute best fit transform
        let (rotation, translation) = compute_best_fit_transform(&transformed_source, &correspondences);

        // Update total rotation and translation
        r_total = rotation * r_total;
        t_total = rotation * t_total + translation;

        // Compute error (e.g., sum of squared distances)
        let error: f32 = transformed_source.iter()
            .zip(correspondences.iter())
            .map(|(s, t)| (s - t).norm_squared())
            .sum();

        if error < 1e-6 {
            break; // Convergence criterion
        }
    }

    (r_total, t_total)
}

fn find_correspondences(source: &[Vector3<f32>], target: &[Vector3<f32>]) -> Vec<Vector3<f32>> {
    // Implement nearest neighbor search or any other method to find correspondences
    let mut correspondences = Vec::with_capacity(source.len());

    for s in source {
        let mut closest_point = target[0];
        let mut min_distance = (s - &target[0]).norm_squared();
        for t in target.iter().skip(1) {
            let distance = (s - t).norm_squared();
            if distance < min_distance {
                min_distance = distance;
                closest_point = *t;
            }
        }
        correspondences.push(closest_point);
    }

    correspondences
}
