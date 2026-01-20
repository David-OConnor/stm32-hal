fn main() {
    // Link embedded-test file only when running `cargo test`
    println!("cargo::rustc-link-arg-tests=-Tembedded-test.x");
}
