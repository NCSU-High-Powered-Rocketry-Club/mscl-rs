// TODO: Delete this file (this is just for testing purposes)

mod parser;
mod structs;
use parser::SerialParser;

fn main() -> std::io::Result<()> {
    let mut parser = SerialParser::new("/dev/ttyACM0", 115200, std::time::Duration::from_secs(1))?;

    loop {
        // println!("Reading packets...");
        let packets = parser.get_all_packets();
        println!("{:?}", packets);
    }
}