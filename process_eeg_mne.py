import pandas as pd
import numpy as np
import mne
import matplotlib.pyplot as plt
from datetime import datetime

# 読み込むファイルパス (保存された最新のCSVを指定してください)
FILE_PATH = r"eeg_data/eeg_data_20260114_144738.csv"

# サンプリングレート
SFREQ = 250

def load_and_process_eeg(csv_path):
    # --- 1. CSV読み込みとデータ整形 ---
    print(f"Loading: {csv_path}")
    df = pd.read_csv(csv_path)

    # EEGデータ列の抽出 ('(uV)' を含む列)
    eeg_cols = [col for col in df.columns if '(uV)' in col]
    data_uv = df[eeg_cols].values.T
    
    # uV (マイクロボルト) -> V (ボルト) 変換
    data_v = data_uv * 1e-6
    
    # チャンネル名リスト
    ch_names = [col.replace(' (uV)', '').strip() for col in eeg_cols]
    
    # Info作成
    info = mne.create_info(ch_names=ch_names, sfreq=SFREQ, ch_types='eeg')
    
    # 時刻設定
    if 'timestamp' in df.columns:
        try:
            start_time_str = df['timestamp'].iloc[0]
            # タイムゾーン等の微調整が必要な場合はここで行います
            meas_date = datetime.fromisoformat(start_time_str).astimezone()
            info.set_meas_date(meas_date)
        except Exception:
            pass

    # Rawオブジェクト作成
    raw = mne.io.RawArray(data_v, info)

    # トリガー情報の統合
    if 'trigger' in df.columns:
        trigger_data = df['trigger'].values
        stim_data = np.zeros((1, len(raw.times)))
        stim_data[0, :] = trigger_data
        stim_info = mne.create_info(['STI'], sfreq=SFREQ, ch_types=['stim'])
        stim_raw = mne.io.RawArray(stim_data, stim_info)
        raw.add_channels([stim_raw], force_update_info=True)
        
        # イベント抽出 (変化点検出)
        events = mne.find_events(raw, stim_channel='STI', output='step', consecutive=True, min_duration=0.005)
    else:
        events = None

    # --- 2. 前処理 (フィルタリング・再参照) ---
    print("Applying processing pipeline...")
    
    # 比較用にRawデータをコピー
    raw_filtered = raw.copy()

    # 【重要】再参照 (Re-referencing)
    # 生データは「回路のGND/Bias」基準の場合が多いため、
    # 全電極の平均を基準にする「Average Reference」が一般的によく使われます。
    # (特定の耳朶電極などがある場合は raw_filtered.set_eeg_reference(['A1', 'A2']) のように指定)
    raw_filtered.set_eeg_reference('average', projection=True)
    raw_filtered.apply_proj()

    # --- フィルタ設定の見直し ---
    # 生データには DCオフセット(0Hz付近) や ドリフト が含まれるため、ハイパスフィルタが必須です。

    # 1. ハイパスフィルタ (1.0 Hz)
    #    ドリフトを除去し、基線を安定させます。
    #    事象関連電位(ERP)を見るなら0.1Hz、通常の波形観察なら1.0Hz程度が安定します。
    raw_filtered.filter(l_freq=0.5, h_freq=None, fir_design='firwin')

    # 2. ノッチフィルタ (50 Hz)
    #    電源周波数ノイズを除去 (東京は50Hz)。
    raw_filtered.notch_filter(freqs=50.0)

    # 3. ローパスフィルタ (40 Hz)
    #    筋電位(EMG)などの高周波ノイズを抑え、波形を滑らかにします。
    #    α波(8-13Hz)やβ波(13-30Hz)を見たい場合、40Hz程度で切ると綺麗に見えます。
    raw_filtered.filter(l_freq=None, h_freq=40.0, fir_design='firwin')

    return raw, raw_filtered, events

if __name__ == "__main__":
    # 処理実行
    raw_original, raw_clean, events = load_and_process_eeg(FILE_PATH)

    # --- 3. 可視化 ---
    # 背景色を白、文字を黒にする設定 (論文やレポート向け)
    fig_kwargs = dict(scalings='auto', title="Processed EEG", show=True, block=True)

    # フィルタ後のきれいな波形をプロット
    # キーボードの 'a' を押すと全チャンネル表示になります
    raw_clean.plot(events=events, duration=10, n_channels=len(raw_clean.ch_names), **fig_kwargs)

    # (オプション) PSD (周波数スペクトル) の比較
    # フィルタが効いているか確認できます
    fig, ax = plt.subplots(2, 1, figsize=(10, 8))
    
    raw_original.compute_psd(fmax=80).plot(axes=ax[0], show=False)
    ax[0].set_title("Original Raw PSD (Drift & Noise)")
    
    raw_clean.compute_psd(fmax=80).plot(axes=ax[1], show=False)
    ax[1].set_title("Filtered PSD (Clean)")
    
    plt.tight_layout()
    plt.show()