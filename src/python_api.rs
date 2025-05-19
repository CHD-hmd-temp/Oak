use crate::interface::render_with_screencapture;
use pyo3::prelude::*;

#[pyfunction]
fn load_config_in_python(config_path: &str) -> PyResult<crate::config::OakConfig> {
    let config = crate::config::load_config(config_path);
    match config {
        Ok(config) => Ok(config),
        Err(e) => Err(pyo3::exceptions::PyRuntimeError::new_err(format!(
            "Error loading config: {}",
            e
        ))),
    }
}

#[pyfunction]
fn run_bevy_with_screencapture(oak_config_path: &str) {
    // Load the configuration
    let oak_config = load_config_in_python(oak_config_path).unwrap_or_else(|_| {
        eprintln!("Error loading config, using default.");
        crate::config::OakConfig::default()
    });
    render_with_screencapture::run_bevy(oak_config.clone());
}

#[pymodule]
fn oak(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<crate::config::OakConfig>()?;
    m.add_function(wrap_pyfunction!(load_config_in_python, m)?)?;
    m.add_function(wrap_pyfunction!(run_bevy_with_screencapture, m)?)?;
    Ok(())
}