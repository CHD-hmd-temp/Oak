// use linfa_clustering::{Dbscan, DbscanParams};
// use ndarray::Array2;
// use crate::prelude::*;

// pub fn cluster_points(points: &[LaserPoint], eps: f32, min_points: usize) -> Vec<LaserPoint> {
//     // 1. 转换为 Array2<f32> 供 DBSCAN 使用
//     let mut data = Array2::<f32>::zeros((points.len(), 3));
//     for (i, p) in points.iter().enumerate() {
//         data[[i, 0]] = p.coordinate.x;
//         data[[i, 1]] = p.coordinate.y;
//         data[[i, 2]] = p.coordinate.z;
//     }

//     // 2. 应用 DBSCAN，使用正确的方法名
//     let dbscan_result = DbscanParams::new(eps)
//         .min_samples(min_points)  // 使用 min_samples 代替 min_points
//         .fit(&data)
//         .expect("DBSCAN failed");

//     // 3. 筛选掉 label == None 的噪声点
//     let labels = dbscan_result.labels();
//     let clustered_points: Vec<LaserPoint> = points
//         .iter()
//         .zip(labels.iter())
//         .filter_map(|(point, &label)| {
//             if label != -1 { Some(*point) } else { None }  // -1 通常表示噪声点
//         })
//         .collect();

//     clustered_points
// }