use crate::prelude::*;
use rand::prelude::IndexedRandom;
use rand::rng;

pub fn ransac_ground_segmentation(
    points: &[LaserPoint],
    iterations: usize,
    distance_threshold: f32,
) -> (Vec<LaserPoint>, Vec<LaserPoint>) {
    let mut best_inliers = Vec::new();
    let mut rng = rng();

    for _ in 0..iterations {
        // 随机选择 3 个不同点
        let sample: Vec<_> = points.choose_multiple(&mut rng, 3).collect();
        if sample.len() < 3 {
            continue;
        }

        let p1 = sample[0].coordinate;
        let p2 = sample[1].coordinate;
        let p3 = sample[2].coordinate;

        // 计算法向量
        let v1 = p2 - p1;
        let v2 = p3 - p1;
        let normal = v1.cross(&v2);

        if normal.norm() == 0.0 {
            continue; // 共线点，跳过
        }

        let (a, b, c) = (normal.x, normal.y, normal.z);
        let d = -normal.dot(&p1.coords);

        // 计算所有点到平面的距离
        let mut inliers = Vec::new();
        for &point in points {
            let p = point.coordinate;
            let dist = (a * p.x + b * p.y + c * p.z + d).abs() / normal.norm();

            if dist < distance_threshold {
                inliers.push(point);
            }
        }

        if inliers.len() > best_inliers.len() {
            best_inliers = inliers;
        }
    }

    // 输出结果
    let ground_points = best_inliers;
    //let ground_set: std::collections::HashSet<_> = ground_points.iter().map(|p| p.coordinate).collect();

    let non_ground_points: Vec<_> = points
    .iter()
    .cloned()
    .filter(|p| {
        !ground_points.iter().any(|g| {
            // Approximate equality check
            let diff = p.coordinate - g.coordinate;
            diff.norm_squared() < 1e-10 // Adjust epsilon as needed
        })
    })
    .collect();

    (ground_points, non_ground_points)
}

#[allow(unused)]
use kiddo::{KdTree, SquaredEuclidean};
/// Parameters for shrub (low vegetation) filtering.
#[allow(unused)]
pub struct ShrubFilterConfig {
    /// Maximum distance between neighboring points to be considered in the same cluster (meters).
    pub cluster_radius: f32,
    /// Minimum number of points for a cluster to be considered non-shrub.
    pub min_cluster_size: usize,
    /// Maximum height above ground to be considered shrub (meters).
    pub max_shrub_height: f32,
}

/// Filters out small, low-height clusters (e.g., shrubs) from a point cloud.
///
/// # Arguments
/// * `points` - input point cloud
/// * `ground_height` - estimated ground z-value (e.g., via ground plane fit)
/// * `config` - filtering parameters
///
/// # Returns
/// A `Vec<LaserPoint>` without low, sparse clusters.
fn _dead() {

}
// pub fn remove_shrubbery(
//     points: &Vec<LaserPoint>,
//     ground_height: f32,
//     config: &ShrubFilterConfig,
// ) -> Vec<LaserPoint> {
//     let mut tree = KdTree::<f32, u64, 3, SquaredEuclidean>::new();
//     // Insert points into KD-tree with 3D coordinates
//     for (i, point) in points.iter().enumerate() {
//     tree.add(
//         &[point.coordinate.x, point.coordinate.y, point.coordinate.z],
//         i as u64
//     );
// }

//     let mut visited = vec![false; points.len()];
//     let mut output = Vec::with_capacity(points.len());

//     for i in 0..points.len() {
//         if visited[i] {
//             continue;
//         }
//         // Region growing for cluster starting at i
//         let mut cluster_indices = Vec::new();
//         let mut stack = vec![i];
//         visited[i] = true;

//         while let Some(idx) = stack.pop() {
//             cluster_indices.push(idx);
//             let p = &points[idx];
//             // Search neighbors within cluster_radius
//             let neighbors = tree
//                 .within(&[p.coordinate.x, p.coordinate.y, p.coordinate.z], config.cluster_radius);
//             for neighbor in neighbors {
//                 let j = neighbor.item as usize;
//                 if !visited[j] {
//                     visited[j] = true;
//                     stack.push(j);
//                 }
//             }
//         }

//         // Analyze cluster
//         let cluster_size = cluster_indices.len();
//         let max_z = cluster_indices.iter()
//             .map(|&j| points[j].coordinate.z)
//             .fold(f32::MIN, f32::max);

//         // If cluster is large enough or tall enough above ground, keep it
//         if cluster_size >= config.min_cluster_size
//             || (max_z - ground_height) > config.max_shrub_height
//         {
//             // retain all points in this cluster
//             for &j in cluster_indices.iter() {
//                 output.push(points[j]);
//             }
//         }
//         // else drop this cluster (shrub)
//     }

//     output
// }

// // Squared Euclidean distance function for kiddo
// fn squared_euclid(a: &[f32; 3], b: &[f32; 3]) -> f32 {
//     (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)
// }
