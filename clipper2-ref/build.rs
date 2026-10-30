use std::{env, fs, path::PathBuf};

const SOURCES: [&str; 4] = [
    "clipper.engine.cpp",
    "clipper.offset.cpp",
    "clipper.rectclip.cpp",
    "clipper.triangulation.cpp",
];

// The only deliberate deviation from upstream: std::sort leaves the order of equal intersection
// nodes to the standard library, so results differ between libc++ and libstdc++. clipper2-rs sorts
// stably; the reference must too.
const UNSTABLE_SORT: &str =
    "std::sort(intersect_nodes_.begin(), intersect_nodes_.end(), IntersectListSort);";
const STABLE_SORT: &str =
    "std::stable_sort(intersect_nodes_.begin(), intersect_nodes_.end(), IntersectListSort);";

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let lib = manifest.join("../upstream/Clipper2-2.0.1/CPP/Clipper2Lib");
    let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("clipper2-src");
    fs::create_dir_all(&out).unwrap();

    let mut sources = Vec::new();
    for name in SOURCES {
        let mut text = fs::read_to_string(lib.join("src").join(name)).unwrap();
        if name == "clipper.engine.cpp" {
            assert!(
                text.contains(UNSTABLE_SORT),
                "upstream intersect sort not found"
            );
            text = text.replace(UNSTABLE_SORT, STABLE_SORT);
        }
        let path = out.join(name);
        fs::write(&path, text).unwrap();
        sources.push(path);
    }
    let shim = manifest.join("shim/ref_shim.cpp");

    for (z, name) in [(false, "clipper2_ref"), (true, "clipper2_ref_z")] {
        let mut build = cc::Build::new();
        build
            .cpp(true)
            .std("c++17")
            .opt_level(2)
            .flag_if_supported("-ffp-contract=off")
            .include(lib.join("include"))
            .files(&sources)
            .file(&shim)
            .warnings(false);
        if z {
            build
                .define("USINGZ", None)
                .define("Clipper2Lib", "Clipper2LibZ");
        }
        build.compile(name);
    }

    println!("cargo:rerun-if-changed=shim/ref_shim.cpp");
    println!("cargo:rerun-if-changed=../upstream/Clipper2-2.0.1/CPP/Clipper2Lib");
}
