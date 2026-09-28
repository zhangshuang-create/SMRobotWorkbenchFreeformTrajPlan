#include "TrajPlanPipeline.h"
#include <iostream>
#include <string>
#include <filesystem>

using namespace SprayTrajPlan;

void printUsage(const char* program_name)
{
    std::cout << "Usage: " << program_name << " [OPTIONS] <input_mesh>\n\n";
    std::cout << "Options:\n";
    std::cout << "  --spacing <value>         Target trajectory spacing (default: 30.0 mm)\n";
    std::cout << "  --sample-step <value>     Sample step size (default: spacing/4)\n";
    std::cout << "  --speed <value>           Trajectory speed (default: 300.0 mm/s)\n";
    std::cout << "  --spray-distance <value>  Spray distance (default: 0.0 mm)\n";
    std::cout << "  --a-max <value>           Maximum acceleration (default: 1000.0 mm/s²)\n";
    std::cout << "  --j-max <value>           Maximum jerk (default: 5000.0 mm/s³)\n";
    std::cout << "  --arc-center-limit <val>  Arc center limit (default: 0.0)\n";
    std::cout << "  --arc-lock-guard <value>  Arc lock guard distance (default: 2.0)\n";
    std::cout << "  --trajectory-out <path>   Output CSV path\n";
    std::cout << "  --metadata-out <path>     Output metadata JSON path\n";
    std::cout << "  --paq-out <path>          Output PAQ trajectory path\n";
    std::cout << "  --optcuts-exe <path>      Path to OptCuts executable\n";
    std::cout << "  --optcuts-root <path>     Path to OptCuts root directory\n";
    std::cout << "  --help                    Display this help message\n";
    std::cout << "\n";
    std::cout << "Example:\n";
    std::cout << "  " << program_name << " --spacing 25.0 --speed 250.0 model.stl\n";
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    // Parse command-line arguments
    TrajectoryParams params;
    std::string input_mesh;
    std::string output_csv;
    std::string output_metadata;
    std::string output_paq;
    std::string optcuts_exe;
    std::string optcuts_root;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "--spacing" && i + 1 < argc) {
            params.target_spacing = std::stod(argv[++i]);
        }
        else if (arg == "--sample-step" && i + 1 < argc) {
            params.sample_step = std::stod(argv[++i]);
        }
        else if (arg == "--speed" && i + 1 < argc) {
            params.trajectory_speed = std::stod(argv[++i]);
        }
        else if (arg == "--spray-distance" && i + 1 < argc) {
            params.spray_distance = std::stod(argv[++i]);
        }
        else if (arg == "--a-max" && i + 1 < argc) {
            params.a_max = std::stod(argv[++i]);
        }
        else if (arg == "--j-max" && i + 1 < argc) {
            params.j_max = std::stod(argv[++i]);
        }
        else if (arg == "--arc-center-limit" && i + 1 < argc) {
            params.arc_center_limit = std::stod(argv[++i]);
        }
        else if (arg == "--arc-lock-guard" && i + 1 < argc) {
            params.arc_lock_guard_d = std::stod(argv[++i]);
        }
        else if (arg == "--trajectory-out" && i + 1 < argc) {
            output_csv = argv[++i];
        }
        else if (arg == "--metadata-out" && i + 1 < argc) {
            output_metadata = argv[++i];
        }
        else if (arg == "--paq-out" && i + 1 < argc) {
            output_paq = argv[++i];
        }
        else if (arg == "--optcuts-exe" && i + 1 < argc) {
            optcuts_exe = argv[++i];
        }
        else if (arg == "--optcuts-root" && i + 1 < argc) {
            optcuts_root = argv[++i];
        }
        else if (arg[0] != '-') {
            input_mesh = arg;
        }
        else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    // Validate input
    if (input_mesh.empty()) {
        std::cerr << "Error: Input mesh file not specified" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    if (!std::filesystem::exists(input_mesh)) {
        std::cerr << "Error: Input mesh file does not exist: " << input_mesh << std::endl;
        return 1;
    }

    // Generate default output paths if not specified
    std::filesystem::path mesh_path(input_mesh);
    std::string base_name = mesh_path.stem().string();

    if (output_csv.empty()) {
        output_csv = base_name + "_optcuts_trajectory.csv";
    }
    if (output_metadata.empty()) {
        output_metadata = base_name + "_optcuts_trajectory.json";
    }
    if (output_paq.empty()) {
        output_paq = base_name + "_optcuts_trajectory_PAQ.txt";
    }

    // Calculate sample step if not specified
    if (params.sample_step <= 0.0) {
        params.sample_step = params.target_spacing / 4.0;
    }

    // Create and configure pipeline
    TrajPlanPipeline pipeline;

    if (!pipeline.initialize()) {
        std::cerr << "Failed to initialize pipeline" << std::endl;
        return 1;
    }

    if (!optcuts_exe.empty()) {
        pipeline.setOptCutsExecutable(optcuts_exe);
    }
    if (!optcuts_root.empty()) {
        pipeline.setOptCutsRoot(optcuts_root);
    }

    // Run pipeline
    std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   Seam-Aware Trajectory Planning Pipeline         ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════╝" << std::endl;

    try {
        if (!pipeline.runPipeline(input_mesh, output_csv, output_metadata, output_paq, params)) {
            std::cerr << "\nPipeline execution failed" << std::endl;
            return 1;
        }

        // Print summary
        const auto& metadata = pipeline.getMetadata();
        std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║   Pipeline Summary                                 ║" << std::endl;
        std::cout << "╚════════════════════════════════════════════════════╝" << std::endl;
        std::cout << "Input mesh:        " << input_mesh << std::endl;
        std::cout << "Output CSV:        " << output_csv << std::endl;
        std::cout << "Output metadata:   " << output_metadata << std::endl;
        std::cout << "Output PAQ:        " << output_paq << std::endl;
        std::cout << "\nTrajectory Parameters:" << std::endl;
        std::cout << "  Spacing:         " << params.target_spacing << " mm" << std::endl;
        std::cout << "  Speed:           " << params.trajectory_speed << " mm/s" << std::endl;
        std::cout << "  Spray distance:  " << params.spray_distance << " mm" << std::endl;
        std::cout << "\nResults:" << std::endl;
        std::cout << "  Total segments:  " << metadata.total_segments << std::endl;
        std::cout << "  Total length:    " << metadata.total_length << " mm" << std::endl;
        std::cout << "\n✓ Trajectory planning completed successfully!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\nException occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
