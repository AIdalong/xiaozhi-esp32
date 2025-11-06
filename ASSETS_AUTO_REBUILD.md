# Assets 自动重建功能

## 概述

项目已经配置了自动重建功能，当你添加、修改或删除 `assets` 目录中的音频文件时，系统会自动检测变化并重新生成配置文件。

## 工作原理

1. **自动扫描**：每次构建时，CMake 会自动扫描 `assets/` 目录下的所有 `.p3` 文件
2. **依赖检测**：系统会检测音频文件的变化，并自动重新生成 `assets/lang_config.h`
3. **增量构建**：只有发生变化的文件才会被重新处理

## 支持的目录

- `main/assets/common/` - 公共音效文件
- `main/assets/zh-CN/` - 中文音效文件
- `main/assets/zh-TW/` - 繁体中文音效文件
- `main/assets/ja-JP/` - 日语音效文件
- `main/assets/en-US/` - 英文音效文件

## 使用方法

### 添加新音频文件

1. 将 `.p3` 文件放入相应目录
2. 运行构建命令：
   ```bash
   idf.py build
   ```

### 修改现有音频文件

1. 替换或修改 `.p3` 文件
2. 运行构建命令：
   ```bash
   idf.py build
   ```

### 删除音频文件

1. 删除 `.p3` 文件
2. 运行构建命令：
   ```bash
   idf.py build
   ```

## 当前配置

### CMakeLists.txt 中的关键配置

```cmake
# 自动扫描音频文件
file(GLOB LANG_SOUNDS ${CMAKE_CURRENT_SOURCE_DIR}/assets/${LANG_DIR}/*.p3)
file(GLOB COMMON_SOUNDS ${CMAKE_CURRENT_SOURCE_DIR}/assets/common/*.p3)

# 创建文件列表用于依赖检测
set(ALL_ASSETS_FILES ${LANG_SOUNDS} ${COMMON_SOUNDS})

# 生成规则，包含所有音频文件作为依赖
add_custom_command(
    OUTPUT ${LANG_HEADER}
    COMMAND python ${PROJECT_DIR}/scripts/gen_lang.py
            --input "${LANG_JSON}"
            --output "${LANG_HEADER}"
    DEPENDS
        ${LANG_JSON}
        ${PROJECT_DIR}/scripts/gen_lang.py
        ${ALL_ASSETS_FILES}
    COMMENT "Generating ${LANG_DIR} language config (auto-rebuild enabled)"
)
```

## 验证功能

构建时会在控制台看到类似输出：

```
-- Found language sounds: /path/to/assets/zh-CN/welcome.p3;/path/to/assets/zh-CN/activation.p3
-- Found common sounds: /path/to/assets/common/success.p3;/path/to/assets/common/exclamation.p3
-- Generating zh-CN language config (auto-rebuild enabled)
```

## 故障排除

### 问题1：新文件没有被检测到

**解决方案**：手动触发重新配置
```bash
idf.py reconfigure
idf.py build
```

### 问题2：构建仍然失败

**解决方案**：清理缓存后重新构建
```bash
idf.py clean
idf.py build
```

### 问题3：代码中无法使用新音效

**检查事项**：
- 确保使用正确的命名格式：`Lang::Sounds::P3_文件名大写`
- 检查 `assets/lang_config.h` 是否包含新文件的定义
- 确保文件扩展名是 `.p3`

## 手动触发重新构建

如果自动检测没有生效，可以使用以下命令手动触发：

```bash
# 方法1：重新配置
idf.py reconfigure
idf.py build

# 方法2：清理后重新构建
idf.py clean
idf.py build

# 方法3：强制重新生成语言配置
rm -f main/assets/lang_config.h
idf.py build
```

## 优势

1. **自动检测**：系统自动检测文件变化
2. **增量构建**：只处理变化的文件，构建速度更快
3. **实时更新**：修改文件后立即生效
4. **调试信息**：构建时显示找到的音频文件

## 注意事项

- 确保音频文件格式正确（P3格式）
- 文件名不要包含特殊字符
- 建议使用小写字母和下划线命名文件
- 如果自动检测不工作，使用手动触发方法

## 技术说明

当前配置使用 CMake 的 `file(GLOB ...)` 和 `add_custom_command` 来实现自动重建：

1. **文件扫描**：`file(GLOB ...)` 在配置时扫描音频文件
2. **依赖管理**：`add_custom_command` 将音频文件作为依赖项
3. **自动生成**：当依赖文件变化时，自动重新生成 `lang_config.h`

这种方法在大多数情况下都能正常工作，但在某些情况下可能需要手动触发重新配置。
