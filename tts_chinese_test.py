import pyttsx3
import sys

print("--- pyttsx3 中文语音测试 ---")

try:
    # 1. 初始化 TTS 引擎
    print("正在初始化 TTS 引擎...")
    engine = pyttsx3.init()
    print("引擎初始化成功。")

    # 2. (可选) 列出所有可用的语音引擎
    print("\n--- 可用的语音引擎 ---")
    voices = engine.getProperty('voices')
    if not voices:
        print("未找到可用的语音引擎。")
    else:
        found_chinese = False
        for i, voice in enumerate(voices):
            print(f"语音 {i}:")
            print(f"  ID: {voice.id}")
            print(f"  Name: {voice.name}")
            # 尝试获取语言属性 (可能不存在或格式不同)
            try:
                 # pyttsx3 v2.7+ 应该有 languages 属性 (列表)
                 langs = getattr(voice, 'languages', [])
                 print(f"  Languages: {langs}")
                 if any('hak' in lang.lower() for lang in langs):
                     found_chinese = True
            except Exception:
                 # 如果 languages 属性或其他检查失败，尝试从 name 判断
                 if 'chinese' in voice.name.lower() or 'mandarin' in voice.name.lower():
                      found_chinese = True
                 print("  (无法准确获取语言列表)")

            try:
                gender = getattr(voice, 'gender', 'N/A')
                print(f"  Gender: {gender}")
            except Exception: pass
            try:
                 age = getattr(voice, 'age', 'N/A')
                 print(f"  Age: {age}")
            except Exception: pass
            print("-" * 10)

        if found_chinese:
            print("\n提示: 检测到可能支持中文的语音 (请检查上面列表中的 Name 或 Languages)。")
            print("你可以尝试通过修改下面代码中的 voice_id 来指定使用中文语音。")
        else:
            print("\n警告: 未明确检测到中文语音支持。将使用系统默认语音尝试朗读中文。")

    # 3. (可选) 尝试设置特定的语音 (如果需要)
    # -- 取消注释并修改下面的 voice_id 为你想要测试的语音 ID ---
    # target_voice_id = None # 设置为 None 使用默认
    # # 例如, 如果你看到一个中文语音的 ID 是 'mandarin' 或某个特定路径:
    # # target_voice_id = 'mandarin'
    # # target_voice_id = 'HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Speech\\Voices\\Tokens\\TTS_MS_ZH-CN_HUIHUI_11.0' # Windows 示例
    # # target_voice_id = 'com.apple.speech.synthesis.voice.ting-ting.premium' # macOS 示例
    #
    # if target_voice_id:
    #     try:
    #         print(f"\n尝试设置语音 ID: {target_voice_id}")
    #         engine.setProperty('voice', target_voice_id)
    #         print("设置成功 (如果 ID 有效)。")
    #     except Exception as e:
    #         print(f"设置语音 ID 时出错: {e}. 将使用默认语音。")
    # else:
    #      print("\n将使用系统默认语音。")

    # 可选：调整语速和音量
    rate = engine.getProperty('rate')
    volume = engine.getProperty('volume')
    print(f"\n当前语速: {rate}, 音量: {volume}")
    # engine.setProperty('rate', rate - 50) # 减慢语速
    # engine.setProperty('volume', 0.9)     # 减小音量

    # 4. 定义要朗读的中文文本
    text_to_speak = "你好，世界。这是中文语音测试。 Hello world, this is a test."
    print(f"\n准备朗读: '{text_to_speak}'")

    # 5. 朗读文本
    engine.say(text_to_speak)

    # 6. 等待朗读完成
    print("正在朗读...")
    engine.runAndWait()
    print("朗读结束。")

    # 7. （可选）测试第二次朗读，有时第一次调用后状态会改变
    # text_to_speak_2 = "第二次测试。"
    # print(f"\n准备朗读: '{text_to_speak_2}'")
    # engine.say(text_to_speak_2)
    # print("正在朗读...")
    # engine.runAndWait()
    # print("朗读结束。")


except ImportError:
    print("\n错误: 未找到 'pyttsx3' 库。")
    print("请使用 'pip install pyttsx3' 进行安装。")
except Exception as e:
    print(f"\n运行过程中发生错误: {e}")
    traceback.print_exc()

print("\n--- 测试结束 ---")