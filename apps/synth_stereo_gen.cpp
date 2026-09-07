// 生成"单帧"合成双目图像对 + 一张真值（ground truth）深度网格，供光学基线
// （optical baseline）评测使用。刻意与 apps/synth_bag_gen.cpp 相互独立：那个工具
// 造的是一整条轨迹 / 位姿图的 bag，这里只造一个静态场景的单帧，用来单独考核
// "双目几何 → 深度"这一段。
//
// 为什么用"合成"而不是直接录一段仿真数据？不是因为跑不了仿真器——WSL2 里装了
// ROS2 Jazzy（~/ros2_ws 里有 holoocean-ros 三个包），宿主 Windows 上的
// UE5 + HoloOcean 以及 Windows <-> WSL2 的 ROS2 桥接在 2026-09-03 已经实测跑通。
// 真正的理由是这里需要的输入有三个仿真录制给不了的性质：
//   (a) 视差与深度是构造出来的、精确已知，真值误差为零，评测端才能拿 rmse 做
//       阈值门禁（tests/integration/optical_baseline_smoke_test.sh 卡 0.05m）；
//   (b) 逐字节可复现，没有任何随机性，回归对比才有意义；
//   (c) 几毫秒跑完、不拉起任何外部进程，CI 与本地 ctest 里才用得起。
// 仿真录制适合验"能不能吃真实数据"，不适合当这类几何精度门禁的输入。
//
// 写出的 topic：
//   /raw/camera/left     uw.domain.ImageFrame
//   /raw/camera/right    uw.domain.ImageFrame
//   /gt/depth            uw.domain.MeasurementEvidence（内含
//                        OpticalDepthPriorMeasurement，producer_type="ground_truth"）
//                        —— 真值复用既有的"米制深度先验"契约，而不是另起一套平行的
//                        GT schema：一份完美的米制深度先验，本来就等价于真值。
//
// 可选副产物（--png-prefix）：把左右目另存成两个 PNG，方便肉眼直接看一眼纹理和
// 平移量对不对。只是给人看的旁路，不参与任何评测链路。
//
// ============================ 本文件主要逻辑 ============================
//
// 一句话：按命令行参数造一对"仅有水平像素平移"的纹理图，再配一张处处等于常数的
// 深度真值，一起写进一个 MCAP 文件。整体是线性的四步，没有任何随机性，因此输出
// 逐字节可复现（无需 seed）。
//
// 1) 解析命令行参数（都有默认值，可整份省略）：
//      --out          输出 MCAP 路径          默认 /tmp/synthetic_stereo.mcap
//      --width/--height 图像尺寸              默认 640x480
//      --disparity-px 右图相对左图的视差(px)  默认 8
//      --depth-m      真值深度(m)             默认 6.3
//      --png-prefix   另存 PNG 预览的路径前缀  默认空（不导出）
//
// 2) 造图（MakeImage）：像素值由 Texture(u, v) = (131u + 67v + 19) mod 256 给出。
//    别被"看起来像哈希"骗了，它是个线性同余式，导出 PNG 一眼就能看到实际是一片
//    周期性的斜条纹，不是自然纹理。它能用于视差搜索靠的是这条性质：131 与 256
//    互质，所以同一行内连续 256 个像素的灰度值两两不同，块匹配在 256px 窗口内
//    有唯一解。代价是超过一个周期就完全重复（pixel(u, v) == pixel(u + 256, v)），
//    所以 --disparity-px 只在远小于 256 时才安全；默认的 8px 有充足余量。
//    左图取 shift=0，右图取 shift=true_disparity_px：右图第 u 列填的是左图
//    第 u+shift 列的纹理，也就是同一个景物点在右图中左移了 shift 个像素，
//    于是 disparity = u_left - u_right = shift（正视差，符合常规约定）。
//    两张图都标 is_rectified=true：这里是"已经理想校正好"的前提，本工具不模拟
//    畸变、也不模拟相机间的相对旋转。
//
// 3) 造真值深度：整张 OpticalDepthPriorMeasurement 的 depth_m 全部填同一个
//    depth_m，variance 给一个极小值（1e-6），valid_mask 全 1（处处有效）。
//    对应的场景语义是"一面正对相机、距离恒定的平面墙"，与第 2 步"全图统一平移
//    shift 像素"的构造完全自洽。注意这里的 depth_m 是相机 optical frame 下
//    的"正向前"距离（与 PressureDepthMeasurement 那个"正向下"的水深不是一回事，
//    见 CLAUDE.md 的符号约定）。
//
// 4) 写 MCAP 并收尾：三条消息时间戳都写 0（单帧静态场景，没有时间维度）。
//
// 5) 可选：给了 --png-prefix 就顺手把左右目导成 PNG（WritePng）。PNG 编码是本文件
//    里自己实现的最小实现（灰度 8 位 + deflate "stored" 块），不引入任何新依赖：
//    仓库里 OpenCV 只 find_package 了 core/calib3d/imgproc 三个组件，为了看两张图
//    去把 imgcodecs 加进必需依赖并不划算。
//
// —— 使用时的一个坑 ——
// --disparity-px 和 --depth-m 是两个彼此独立的参数，本工具并不会替你校验它们
// 是否自洽；自洽性由 rig 标定决定：depth = fx * baseline / disparity。默认值就是
// 按 configs/rig/example_auv.yaml 配的（fx=420，基线 0.12m，8px → 6.3m）。改动
// 其中任何一个而不同步改另一个，评测端（optical_baseline_eval）算出来的 rmse
// 会直接超阈值——那不是几何管线的 bug，是输入自相矛盾。
// =======================================================================
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

#include "domain/domain.hpp"
#include "runtime/mcap_io.hpp"

namespace {

// 线性同余纹理（不是伪随机，视觉上是斜条纹）：131 与 256 互质 => 同一行内连续
// 256 个像素灰度互不相同，块匹配在这个窗口里有唯一解；但每 256 px 完整重复一次。
uint8_t Texture(int u, int v) { return static_cast<uint8_t>((u * 131 + v * 67 + 19) % 256); }

// 造一张 MONO8 图像；shift 为纹理在水平方向的整体平移量（左图传 0，右图传视差）。
uw::domain::ImageFrame MakeImage(const std::string& frame, uint32_t width, uint32_t height, int shift) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_frame()->set_value(frame);
  image.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  image.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  image.mutable_header()->set_provenance("synth_stereo_gen_v1");
  image.set_width(width);
  image.set_height(height);
  image.set_row_stride_bytes(width);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  std::string pixels(static_cast<std::size_t>(width) * height, '\0');
  for (uint32_t v = 0; v < height; ++v) {
    for (uint32_t u = 0; u < width; ++u) {
      // 右图第 u 列取左图第 u+shift 列的纹理 => disparity = u_left - u_right = shift
      pixels[static_cast<std::size_t>(v) * width + u] =
          static_cast<char>(Texture(static_cast<int>(u) + shift, static_cast<int>(v)));
    }
  }
  image.set_pixel_data(pixels);
  image.set_is_rectified(true);  // 本工具直接给出理想校正图，不模拟畸变与相机间相对旋转
  return image;
}

// ---------------- 以下是给人看的 PNG 旁路（--png-prefix），不参与评测 ----------------
//
// 这里手写一个最小的灰度 PNG 编码器。之所以不复用 OpenCV 的 imwrite：本仓库
// find_package(OpenCV) 只要了 core/calib3d/imgproc，imgcodecs 不在其中；而且这个 app
// 现在只链 uw::domain / uw::runtime，为了导两张预览图而把 OpenCV 拉进它的依赖并不值。
// PNG 格式本身很薄：签名 + IHDR + IDAT + IEND，IDAT 里放一条 zlib 流即可。

void AppendBe32(std::string& out, uint32_t value) {
  out.push_back(static_cast<char>((value >> 24) & 0xFF));
  out.push_back(static_cast<char>((value >> 16) & 0xFF));
  out.push_back(static_cast<char>((value >> 8) & 0xFF));
  out.push_back(static_cast<char>(value & 0xFF));
}

// PNG 每个 chunk 尾部的 CRC-32（多项式 0xEDB88320），逐位算，图不大不必建查表。
uint32_t Crc32(const std::string& data) {
  uint32_t crc = 0xFFFFFFFFu;
  for (unsigned char byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

// zlib 流尾部的 Adler-32 校验（对未压缩的原始字节算）。
uint32_t Adler32(const std::string& data) {
  uint32_t a = 1;
  uint32_t b = 0;
  for (unsigned char byte : data) {
    a = (a + byte) % 65521u;
    b = (b + a) % 65521u;
  }
  return (b << 16) | a;
}

// 组一个 PNG chunk：长度(4) + 类型(4) + 数据 + CRC(4)，CRC 覆盖"类型 + 数据"。
void AppendChunk(std::string& out, const std::string& type, const std::string& payload) {
  AppendBe32(out, static_cast<uint32_t>(payload.size()));
  const std::string body = type + payload;
  out += body;
  AppendBe32(out, Crc32(body));
}

// 把原始字节包成一条 zlib 流，但全部使用 deflate 的 "stored"（BTYPE=00）块——即不压缩。
// 这样不需要链接 zlib，也不需要实现 huffman；代价只是 PNG 文件体积约等于像素数。
// 预览图用途下这点体积无所谓（640x480 约 300KB）。
std::string ZlibStored(const std::string& raw) {
  std::string out;
  out.push_back(static_cast<char>(0x78));  // CMF：deflate，32K 窗口
  out.push_back(static_cast<char>(0x01));  // FLG：无预置字典，最低压缩等级
  constexpr std::size_t kMaxBlock = 65535;
  std::size_t offset = 0;
  do {
    const std::size_t len = std::min(kMaxBlock, raw.size() - offset);
    const bool final_block = (offset + len >= raw.size());
    out.push_back(static_cast<char>(final_block ? 1 : 0));  // BFINAL + BTYPE=00
    // stored 块头里的 LEN / NLEN 是小端，且 NLEN 必须是 LEN 的按位取反
    out.push_back(static_cast<char>(len & 0xFF));
    out.push_back(static_cast<char>((len >> 8) & 0xFF));
    out.push_back(static_cast<char>(~len & 0xFF));
    out.push_back(static_cast<char>((~len >> 8) & 0xFF));
    out.append(raw, offset, len);
    offset += len;
  } while (offset < raw.size());
  AppendBe32(out, Adler32(raw));
  return out;
}

// 把一帧 MONO8 图像写成 8 位灰度 PNG。成功返回 true。
bool WritePng(const std::string& path, const uw::domain::ImageFrame& image) {
  const uint32_t width = image.width();
  const uint32_t height = image.height();

  // PNG 的每一行前面要加一个 filter 字节；这里统一用 0（None），不做行间预测。
  std::string raw;
  raw.reserve(static_cast<std::size_t>(height) * (width + 1));
  for (uint32_t v = 0; v < height; ++v) {
    raw.push_back('\0');
    raw.append(image.pixel_data(), static_cast<std::size_t>(v) * width, width);
  }

  std::string ihdr;
  AppendBe32(ihdr, width);
  AppendBe32(ihdr, height);
  ihdr.push_back(static_cast<char>(8));  // 位深 8
  ihdr.push_back(static_cast<char>(0));  // 颜色类型 0 = 灰度
  ihdr.push_back(static_cast<char>(0));  // 压缩方法 0 = deflate
  ihdr.push_back(static_cast<char>(0));  // 滤波方法 0
  ihdr.push_back(static_cast<char>(0));  // 非隔行

  std::string png("\x89PNG\r\n\x1a\n", 8);  // 固定 8 字节签名
  AppendChunk(png, "IHDR", ihdr);
  AppendChunk(png, "IDAT", ZlibStored(raw));
  AppendChunk(png, "IEND", std::string());

  std::ofstream file(path, std::ios::binary);
  if (!file) {
    return false;
  }
  file.write(png.data(), static_cast<std::streamsize>(png.size()));
  return static_cast<bool>(file);
}

}  // namespace

int main(int argc, char** argv) {
  // 默认值与 configs/rig/example_auv.yaml 配套：fx=420、基线 0.12m，
  // 故 8px 视差对应 420 * 0.12 / 8 = 6.3m。
  std::string out_path = "/tmp/synthetic_stereo.mcap";
  uint32_t width = 640;
  uint32_t height = 480;
  int true_disparity_px = 8;
  double depth_m = 6.3;
  std::string png_prefix;  // 非空时另存 <prefix>_left.png / <prefix>_right.png

  // 手写参数解析：apps/ 层只做参数解析与进程入口，不放编排逻辑。
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() { return std::string(argv[++i]); };
    if (arg == "--out" && i + 1 < argc) {
      out_path = next();
    } else if (arg == "--width" && i + 1 < argc) {
      width = static_cast<uint32_t>(std::stoul(next()));
    } else if (arg == "--height" && i + 1 < argc) {
      height = static_cast<uint32_t>(std::stoul(next()));
    } else if (arg == "--disparity-px" && i + 1 < argc) {
      true_disparity_px = std::stoi(next());
    } else if (arg == "--depth-m" && i + 1 < argc) {
      depth_m = std::stod(next());
    } else if (arg == "--png-prefix" && i + 1 < argc) {
      png_prefix = next();
    } else {
      // 未知参数一律 fail-fast，避免拼错的 flag 被静默忽略、拿到一份参数不对的 bag。
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }

  uw::runtime::McapProtobufWriter writer;
  if (!writer.Open(out_path)) {
    std::cerr << "failed to open " << out_path << " for writing\n";
    return 1;
  }

  // 左右目：唯一差别就是纹理整体平移了 true_disparity_px 个像素。
  const auto left = MakeImage("camera_left_link", width, height, 0);
  const auto right = MakeImage("camera_right_link", width, height, true_disparity_px);
  // 单帧静态场景，三条消息时间戳统一写 0。
  writer.WriteMessage("/raw/camera/left", 0, left);
  writer.WriteMessage("/raw/camera/right", 0, right);

  // 可选的肉眼预览：PNG 只是旁路产物，写失败也不该让整个生成失败（MCAP 才是正品），
  // 所以这里只报错提示、不改退出码。
  if (!png_prefix.empty()) {
    const std::string left_png = png_prefix + "_left.png";
    const std::string right_png = png_prefix + "_right.png";
    if (WritePng(left_png, left) && WritePng(right_png, right)) {
      std::cout << "wrote preview PNGs: " << left_png << " " << right_png << "\n";
    } else {
      std::cerr << "warning: failed to write preview PNGs under prefix " << png_prefix << "\n";
    }
  }

  // 真值深度网格：一面正对相机的等距平面墙，与"全图统一平移"的图像构造自洽。
  uw::domain::OpticalDepthPriorMeasurement gt;
  *gt.mutable_reference_camera_frame() = left.header().sensor_frame();
  gt.set_width(width);
  gt.set_height(height);
  gt.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);  // 米制，可直接与估计值比对
  gt.set_producer_type("ground_truth");
  std::string valid_mask(static_cast<std::size_t>(width) * height, 1);  // 处处有效
  for (uint32_t i = 0; i < width * height; ++i) {
    gt.add_depth_m(static_cast<float>(depth_m));  // 相机 optical frame 下的"正向前"距离
    gt.add_variance_m2(1e-6f);                    // 真值，方差给到接近 0
  }
  gt.set_valid_mask(valid_mask);

  uw::domain::EvidenceId gt_id;
  gt_id.set_value("gt_depth_0");
  auto gt_evidence = uw::domain::MakeEvidence(gt_id, {}, gt, 0.0, "synth_stereo_gen_v1");
  writer.WriteMessage("/gt/depth", 0, gt_evidence);

  writer.Close();
  std::cout << "wrote " << width << "x" << height << " stereo pair (disparity=" << true_disparity_px
            << "px, depth=" << depth_m << "m) to " << out_path << "\n";
  return 0;
}
