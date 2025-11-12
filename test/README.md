
运行测试之前，请先通过源码编译或者包管理器安装的方式安装 Google Test 框架：

```bash
sudo apt update
sudo apt install -y libgtest-dev libgmock-dev
```

编译并运行测试：

```bash
cmake -Bbuild -DBUILD_TESTS=ON
cmake --build build -- -j$(nproc)
build/test/librm_tests
```