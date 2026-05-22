{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";

    mrs_lib_repo.url = "github:ctu-mrs/mrs_lib/nix";
    mrs_lib_repo.inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    mrs_lib_repo.inputs.nix-ros-overlay.follows = "nix-ros-overlay";

    mrs_uav_hw_api_repo.url = "github:ctu-mrs/mrs_uav_hw_api/nix";
    mrs_uav_hw_api_repo.inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    mrs_uav_hw_api_repo.inputs.nix-ros-overlay.follows = "nix-ros-overlay";

    mrs_msgs_repo.url = "github:ctu-mrs/mrs_msgs/nix";
    mrs_msgs_repo.inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    mrs_msgs_repo.inputs.nix-ros-overlay.follows = "nix-ros-overlay";
  };

  outputs = { self, nix-ros-overlay, nixpkgs, mrs_lib_repo, mrs_msgs_repo, mrs_uav_hw_api_repo }:

    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros = pkgs.rosPackages.jazzy;

        mrs_lib_pkg = mrs_lib_repo.packages.${system}.default;
        mrs_msgs_pkg = mrs_msgs_repo.packages.${system}.default;
        mrs_uav_hw_api_pkg = mrs_uav_hw_api_repo.packages.${system}.default;

        # 1. Define ALL dependencies here so the shell and package share them
        simulator_deps = [
          ros.ros-core
          ros.ament-cmake
          ros.ament-cmake-core
          ros.rosidl-default-generators
          ros.builtin-interfaces
          ros.tf2-geometry-msgs
          ros.tf2-eigen
          pkgs.yaml-cpp
          pkgs.boost
          mrs_lib_pkg
          mrs_msgs_pkg
          mrs_uav_hw_api_pkg
        ];

      in {

        # 2. Assign the build to packages.default so Nix recognizes it
        packages.default = ros.buildRosPackage {
          pname = "mrs_multirotor_simulator";
          version = "2.0.0";
          src = ./.;
          buildType = "ament_cmake";
          
          nativeBuildInputs = [ 
            ros.ament-cmake 
            ros.rosidl-default-generators 
          ];

          # PUBLIC dependencies. 
          # These automatically transition to any downstream package.
          propagatedBuildInputs = [ 
            mrs_uav_hw_api_pkg
          ];
        };
          
          buildInputs = simulator_deps;
        };

        # 3. Inject ONLY the dependencies into the shell, not the package itself
        devShells.default = pkgs.mkShell {
          name = "mrs_multirotor_simulator_shell";
          packages = [
            pkgs.colcon
            (ros.buildEnv {
              paths = simulator_deps;
            })
          ];
        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
