use std::process::ExitCode;

fn main() -> ExitCode {
    match raceline_optimizer_cli::run(std::env::args_os().skip(1)) {
        Ok(output) => {
            if let Some(output) = output {
                println!("{output}");
            }
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("{}", error.render());
            ExitCode::from(error.exit_code())
        }
    }
}
