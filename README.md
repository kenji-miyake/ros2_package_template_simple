# ros2_package_template_simple

My ROS2 package template

## Prerequisite

You can install the dependencies using either `brew` or `apt` and `bin`.

### Using brew
```sh
brew install fd sd rename
```

### Or using apt and bin
```sh
sudo apt install fd-find rename

# Then link fdfind as fd, https://github.com/sharkdp/fd#on-ubuntu
ln -s $(which fdfind) ~/.local/bin/fd
```

Install `bin`
```sh
wget https://github.com/marcosnils/bin/releases/download/v0.15.1/bin_0.15.1_Linux_x86_64
chmod +x ./bin_0.15.1_Linux_x86_64
./bin_0.15.1_Linux_x86_64 github.com/marcosnils/bin
# If it asks for download location, enter (full path): /home/yourusername/.local/bin/
```

To update the `$PATH` either reboot or enter `export PATH="$PATH:$HOME/.local/bin/"` in your terminal.

Then, install `sd` using `bin`.
```sh
bin install github.com/chmln/sd ~/.local/bin/sd
# When it asks Multiple matches found, please select one:
# select [1] sd-v0.7.6-x86_64-unknown-linux-gnu
```

## Usage

```sh
cd YOUR_ROS_WORKSPACE
git clone https://github.com/kenji-miyake/ros2_package_template_simple.git YOUR_PACKAGE_NAME
cd YOUR_PACKAGE_NAME

# case1. If you'd like to cleanup manually
./setup.fish
mv PACKAGE_README.md README.md
rm setup.fish
rm -rf .git # When you add this to an existing repository

# case2. If you'd like to cleanup automatically
./setup.fish --clean
```
