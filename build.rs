use cfg_aliases::cfg_aliases;

fn main() {
    cfg_aliases! {
        dma2: { not(any(feature = "f3x4", feature = "f301", feature = "g0", feature = "wb", feature = "c0")) },
        // Only applies to "baseline" clocks. (no H)
        msi: { not(any(feature = "g0", feature = "g4", feature = "c0")) },
    }
}