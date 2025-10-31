# Morthine Xiang's Repository :fireworks:

<details>

**<summary>:musical_note: 劇場 - ヒグチアイ (HiguchiAi)</summary>**
```
ステージの上

一本のスポットライトがさす

客席には二人の男と女

わたしは泣いた

ありったけの力を込めて

それだけで客席は埋まっていく

ステージの上

一本のスポットライトがさす

客席から立ち去る人もいたけど

それ以上に座る人が増えていた

わたしのことを

見て欲しくなった もっと

もう会えない人よ

もう会わないと決めた人よ

あなたの劇場でしあわせでいて

目に入ったもの全てが

わたしの身体の一部になるの

例外はなく あなたも

ステージの上

一本のスポットライトがさす

踊る私に拍手は鳴り止まない

生きる意味は見つけたんだ

見つけたけれど

これがなければ

生きる意味はないとも思った

ステージの上

一本のスポットライトがさす

無表情で立ち去る人が増えた

蛇を飲もうか 服を脱ごうか

血を流そうか なにをしたら

座ったままでいてくれるの

どうか

もう会えない人よ

もう会わないと決めた人よ

あなたの劇場でしあわせでいて

目に入ったもの全てが

わたしの身体の一部になるの

例外はなく あなたも

わたしが存在する意味は

わからないのに

あなたが存在する意味は

こんなに胸に溢れている

出会いや別れを

肯定や否定で色付けしたくない

息をするように 当たり前に

わたしがいる

ステージの上

一本のスポットライトがさす

客席には誰一人座っていない

無音の劇場

わたしは一人 歌い踊る

あなたからもらったもので

わたしはできてる

さみしいけど孤独じゃないの

愛してくれて ありがとう
```
</details>

## :pushpin: Projects

**RobotPilots Cpp Template :toolbox:**
  
  :link:Link: [qianchen36/RP_CppTemplate](https://github.com/qianchen36/RP_CppTemplate) (Already deleted)

**RobotPilots RMUC24 Engineer Renewed :truck:**
  
  :open_file_folder:Path: [/RM2024-Engineer-Renewed](/RM2024-Engineer-Renewed)

  ![RM2024-Engineer-Renewed](/doc/engineer_renewed.jpg)

**RobotPilots RMUC24 Engineer Version-2 :truck:**
  
  :open_file_folder:Path: [/RM2024-Engineer-V2](/RM2024-Engineer-V2)

  ![RM2024-Engineer-V2](/doc/engineer_v2.jpg)

## :pushpin: How to Build

**Environment :package:**

- CMake (v3.20 +) [Go to Website](https://cmake.org/)
- Ninja-Build (v1.11 +) [Go to Website](https://ninja-build.org/)
- Arm GNU Toolchain arm-none-eabi (13.2.rel1 +) [Go to Website](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)

**Step :zero:**

Run these following command line in your terminal (e.g. Command Prompt, Windows Powershell, bash)

`cmake --version`
`ninja --version`
`arm-none-eabi-gcc --version`

If each command return the version info correctly, go to next step. Else, check you software installiation and your `%PATH%` environment variable.

**Step :one:**

Download this repository as ZIP file and unpack it or use `git clone`.

Open a terminal on repository directory.

Use `cd ./[PROJECT_FOLDER]` to enter the project's folder which you want to build.

**Step :two:**

Run `cmake -Bbuild -G "Ninja" | cmake --build build"` in project directory to start building process.

If succeed, the output ELF file is in `./[PROJECT_FOLDER]/build/FIRMWARE.elf`. There also have a hex file and a binary file.
