#include <unitree/robot/go2/vui/vui_client.hpp>

// 主函式，執行 VUI 客戶端測試
int main(int32_t argc, const char** argv)
{
    // 檢查參數數量是否正確
    if (argc < 2)
    {
        std::cout << "Usage: vui_client_example network_interface_name" << std::endl;
        exit(0);
    }

    // 獲取網路介面名稱
    std::string networkInterface = argv[1];
    // 初始化通道工廠，設定網路介面
    unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);
    // 建立 VUI 客戶端實例
    unitree::robot::go2::VuiClient vc;

    // 設定超時時間為 1 秒
    vc.SetTimeout(1.0f);
    // 初始化 VUI 客戶端
    vc.Init();

    // 測試 API
    int level = 0, value = 0;
    int ret;

    while (true)
    {
        // 設定亮度，並輸出返回值
        ret = vc.SetBrightness(level);
        std::cout << "SetBrightness  level=" << level << ", api return:" << ret << std::endl;
        ++level %= 11;  // 亮度級別在 0-10 之間循環
        sleep(1);

        // 獲取亮度值，並輸出返回值
        ret = vc.GetBrightness(value);
        std::cout << "GetBrightness value=" << value << ", api return:" << ret << std::endl;
        sleep(1);
    }

    return 0;
}
