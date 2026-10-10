use air_ground_smart3d_worker::Worker;
use std::io::{self, BufRead, Write};
fn main() {
    let mut worker = Worker::default();
    let input = io::stdin();
    let mut output = io::BufWriter::new(io::stdout());
    for line in input.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => break,
        };
        let result = if line.len() > 128 * 1024 * 1024 {
            json::object! {ok:false,reason:"request_too_large"}
        } else {
            match json::parse(&line) {
                Ok(v) => match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    worker.handle(&v)
                })) {
                    Ok(r) => r,
                    Err(_) => {
                        worker = Worker::default();
                        json::object! {ok:false,reason:"native_panic_session_invalidated",execution_authorized:false}
                    }
                },
                Err(e) => json::object! {ok:false,reason:format!("invalid_json: {e}")},
            }
        };
        if writeln!(output, "{}", result.dump())
            .and_then(|_| output.flush())
            .is_err()
        {
            break;
        }
    }
}
