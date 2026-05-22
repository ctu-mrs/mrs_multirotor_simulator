{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";

    mrs_lib_repo.url = "github:ctu-mrs/mrs_lib/nix";
    mrs_lib_repo.inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    mrs_lib_repo.inputs.nix-ros-overlay.follows = "nix-ros-overlay";

    mrs_msgs_repo.url = "github:ctu-mrs/mrs_msgs/nix";
    mrs_msgs_repo.inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    mrs_msgs_repo.inputs.nix-ros-overlay.follows = "nix-ros-overlay";
  };

  outputs = { self, nix-ros-overlay, nixpkgs, mrs_lib_repo, mrs_msgs_repo }:

    # This automatically loops through x86_64-linux, aarch64-linux, etc.
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros = pkgs.rosPackages.jazzy;

        mrs_lib_pkg = mrs_lib_repo.packages.${system}.default;
        mrs_msgs_pkg = mrs_msgs_repo.packages.${system}.default;
      in {

        # We drop ${system} here because eachDefaultSystem handles it
        packages.default = ros.buildRosPackage {
          pname = "mrs_multirotor_simulator";
          version = "2.0.0";
          
          # Use path syntax, not string syntax
          src = ./.;
          
          buildType = "ament_cmake";
          
          nativeBuildInputs = [ 
            ros.ament-cmake 
            ros.rosidl-default-generators 
          ];
          
          buildInputs = [ 
            ros.ros-core
            ros.ament-cmake-core
            ros.builtin-interfaces
            ros.sensor-msgs
            ros.std-srvs
            ros.std-msgs
            ros.nav-msgs
            ros.geometry-msgs
            ros.python-cmake-module
            ros.rosidl-default-runtime
            ros.tf2
            ros.tf2-geometry-msgs
            ros.tf2-eigen
            ros.visualization-msgs

            pkgs.eigen
            pkgs.yaml-cpp
            pkgs.boost

            mrs_lib_pkg
            mrs_msgs_pkg
          ];
        };

        devShells.default = pkgs.mkShell {
          name = "mrs_multirotor_simulator";
          packages = [
            pkgs.colcon
            (ros.buildEnv {
              paths = [
                ros.ros-core
              ];
            })
          ];
        };
      });

  # This configures Nix to download pre-built ROS binaries instead of compiling C++ from scratch
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
