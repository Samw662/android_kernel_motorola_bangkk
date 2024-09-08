mkdir -p toolchain
cd toolchain
echo 'Download antman and sync'
bash <(curl -s "https://raw.githubusercontent.com/Neutron-Toolchains/antman/main/antman") -S=11032023 # sync neutron clang 17
echo 'Patch for glibc'
bash <(curl -s "https://raw.githubusercontent.com/Neutron-Toolchains/antman/main/antman") --patch=glibc
echo 'Done'

