# ros2_package_template_simple

My ROS2 package template

## Prerequisite

- [bin](https://github.com/marcosnils/bin)

  ```sh
  wget -O /tmp/bin https://github.com/marcosnils/bin/releases/download/v0.15.1/bin_0.15.1_Linux_x86_64
  chmod +x /tmp/bin
  mkdir -p ~/.local/bin
  export PATH=~/.local/bin:$PATH
  /tmp/bin install https://github.com/marcosnils/bin
  ```

- [fd](https://github.com/sharkdp/fd)

  ```sh
  sudo apt install fd-find
  ln -s $(which fdfind) ~/.local/bin/fd
  ```

- [sd](https://github.com/chmln/sd)

  ```sh
  bin install https://github.com/chmln/sd
  # When it asks "Multiple matches found, please select one:",
  # select [1] sd-v0.7.6-x86_64-unknown-linux-gnu
  ```

- [rnr](https://github.com/ismaelgv/rnr)

  ```sh
  bin install https://github.com/ismaelgv/rnr
  # When it asks "Multiple matches found, please select one:",
  # select [1] rnr-v0.4.1-x86_64-unknown-linux-gnu.tar.gz
  # and select [7] rnr-v0.4.1-x86_64-unknown-linux-gnu/rnr
  ```

## Usage

```sh
cd YOUR_ROS_WORKSPACE
git clone https://github.com/kenji-miyake/ros2_package_template_simple.git YOUR_PACKAGE_NAME
cd YOUR_PACKAGE_NAME

# Case1: If you'd like to cleanup manually
./setup.fish
mv PACKAGE_README.md README.md
rm setup.fish
rm -rf .git # When you add this to an existing repository

# Case2: If you'd like to cleanup automatically
./setup.fish --clean
```
