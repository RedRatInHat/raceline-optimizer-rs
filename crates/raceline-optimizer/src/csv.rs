use std::fs;
use std::path::Path;

use crate::contracts::TrajectoryResultSeriesV1;

pub fn read_semicolon_csv(path: impl AsRef<Path>) -> Result<(Vec<String>, Vec<Vec<f64>>), String> {
    let path = path.as_ref();
    let body = fs::read_to_string(path)
        .map_err(|error| format!("failed to read {}: {error}", path.display()))?;
    let mut lines = body.lines().filter(|line| !line.trim().is_empty());
    let header = lines
        .next()
        .ok_or_else(|| format!("empty CSV: {}", path.display()))?
        .trim_start_matches('#')
        .trim();
    let column_names = header
        .split(';')
        .map(|part| part.trim().to_owned())
        .collect::<Vec<_>>();
    let rows = lines
        .map(|line| {
            line.split(';')
                .map(|part| {
                    part.trim().parse::<f64>().map_err(|error| {
                        format!("invalid float `{part}` in {}: {error}", path.display())
                    })
                })
                .collect::<Result<Vec<_>, _>>()
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok((column_names, rows))
}

pub fn read_comma_csv(path: impl AsRef<Path>) -> Result<(Vec<String>, Vec<Vec<f64>>), String> {
    read_delimited_csv(path, ',')
}

fn read_delimited_csv(
    path: impl AsRef<Path>,
    delimiter: char,
) -> Result<(Vec<String>, Vec<Vec<f64>>), String> {
    let path = path.as_ref();
    let body = fs::read_to_string(path)
        .map_err(|error| format!("failed to read {}: {error}", path.display()))?;
    let mut lines = body.lines().filter(|line| !line.trim().is_empty());
    let header = lines
        .next()
        .ok_or_else(|| format!("empty CSV: {}", path.display()))?
        .trim_start_matches('#')
        .trim();
    let column_names = header
        .split(delimiter)
        .map(|part| part.trim().to_owned())
        .collect::<Vec<_>>();
    let rows = lines
        .map(|line| {
            line.split(delimiter)
                .map(|part| {
                    part.trim().parse::<f64>().map_err(|error| {
                        format!("invalid float `{part}` in {}: {error}", path.display())
                    })
                })
                .collect::<Result<Vec<_>, _>>()
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok((column_names, rows))
}

pub fn read_trajectory_result_series(
    path: impl AsRef<Path>,
) -> Result<TrajectoryResultSeriesV1, String> {
    let (columns, rows) = read_semicolon_csv(path)?;
    TrajectoryResultSeriesV1::from_columns(&columns, &rows)
}

#[cfg(test)]
mod tests {
    use std::fs;

    use super::read_semicolon_csv;

    #[test]
    fn reads_hash_prefixed_semicolon_csv() {
        let root = std::env::temp_dir().join(format!("rlc_csv_{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        fs::create_dir_all(&root).unwrap();
        let csv = root.join("sample.csv");
        fs::write(&csv, "# a;b\n1.0;2.5\n").unwrap();

        let (columns, rows) = read_semicolon_csv(&csv).unwrap();

        assert_eq!(columns, vec!["a", "b"]);
        assert_eq!(rows, vec![vec![1.0, 2.5]]);
        let _ = fs::remove_dir_all(root);
    }
}
