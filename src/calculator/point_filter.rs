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