#!/usr/bin/env python3
import os
import sys
import shutil
from pathlib import Path
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


class AnnotatorGUI:
    def __init__(self, root_dir: Path):
        self.root_dir = root_dir.resolve()
        if not self.root_dir.is_dir():
            raise ValueError(f"Not a directory: {self.root_dir}")

        # Папки назначения
        self.safe_dir = self.root_dir / "safe"
        self.danger_dir = self.root_dir / "danger"
        self.safe_dir.mkdir(exist_ok=True)
        self.danger_dir.mkdir(exist_ok=True)

        # Список изображений (только из root_dir, НЕ рекурсивно, и не из safe/danger)
        self.images = self._scan_images()

        self.idx = 0
        self.current_path: Path | None = None
        self.current_imgtk = None  # держим ссылку, чтобы Tkinter не удалял картинку

        # --- UI ---
        self.window = tk.Tk()
        self.window.title("Dataset Annotator (safe / danger)")
        self.window.geometry("1100x700")

        # Левая часть: изображение
        self.img_label = tk.Label(self.window, bd=2, relief="sunken")
        self.img_label.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        # Правая панель: кнопки и инфо
        panel = tk.Frame(self.window)
        panel.pack(side="right", fill="y", padx=10, pady=10)

        self.info_var = tk.StringVar(value="")
        self.info_label = tk.Label(panel, textvariable=self.info_var, justify="left")
        self.info_label.pack(pady=(0, 20))

        self.btn_danger = tk.Button(panel, text="DANGER", width=20, height=3,
                                    command=self.mark_danger, bg="#ff4d4d", fg="white")
        self.btn_danger.pack(pady=10)

        self.btn_safe = tk.Button(panel, text="SAFE", width=20, height=3,
                                  command=self.mark_safe, bg="#39d98a", fg="white")
        self.btn_safe.pack(pady=10)

        self.btn_skip = tk.Button(panel, text="SKIP", width=20, height=2,
                                  command=self.skip)
        self.btn_skip.pack(pady=(30, 10))

        self.btn_undo = tk.Button(panel, text="UNDO (last)", width=20, height=2,
                                  command=self.undo)
        self.btn_undo.pack(pady=10)

        self.window.bind("<KeyPress-d>", lambda e: self.mark_danger())  # d = danger
        self.window.bind("<KeyPress-s>", lambda e: self.mark_safe())    # s = safe
        self.window.bind("<KeyPress-space>", lambda e: self.skip())     # space = skip

        # Для undo
        self.last_move = None  # (src_original_path, dst_path)

        # Показать первое изображение
        self.show_current()

    def _scan_images(self):
        imgs = []
        for p in sorted(self.root_dir.iterdir()):
            if p.is_file() and p.suffix.lower() in IMG_EXTS:
                imgs.append(p)
        return imgs

    def _update_info(self):
        total = len(self.images)
        if total == 0:
            self.info_var.set(
                f"Folder: {self.root_dir}\n\nNo images found."
            )
            return

        remaining = total - self.idx
        current_name = self.current_path.name if self.current_path else "-"
        self.info_var.set(
            f"Folder: {self.root_dir}\n"
            f"Image: {self.idx+1}/{total}\n"
            f"Current: {current_name}\n"
            f"Remaining: {remaining}\n\n"
            f"Hotkeys:\n"
            f"  D = danger\n"
            f"  S = safe\n"
            f"  Space = skip\n"
        )

    def _load_image(self, path: Path):
        # Открываем и подгоняем под окно (без искажения)
        img = Image.open(path).convert("RGB")
        w, h = img.size

        # Размер области под изображение (примерно, учитывая боковую панель)
        max_w = 850
        max_h = 650

        scale = min(max_w / w, max_h / h, 1.0)
        new_w = int(w * scale)
        new_h = int(h * scale)

        img = img.resize((new_w, new_h), Image.Resampling.LANCZOS)
        return ImageTk.PhotoImage(img)

    def show_current(self):
        if self.idx >= len(self.images):
            self.current_path = None
            self.img_label.config(image="")
            self._update_info()
            messagebox.showinfo("Done", "All images have been labeled (or skipped).")
            return

        self.current_path = self.images[self.idx]
        self._update_info()

        try:
            self.current_imgtk = self._load_image(self.current_path)
            self.img_label.config(image=self.current_imgtk)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load {self.current_path.name}\n{e}")
            self.idx += 1
            self.show_current()

    def _move_current(self, target_dir: Path):
        if self.current_path is None:
            return

        src = self.current_path
        dst = target_dir / src.name

        # если файл с таким именем уже есть — добавим суффикс
        if dst.exists():
            stem = dst.stem
            suffix = dst.suffix
            k = 1
            while True:
                candidate = target_dir / f"{stem}_{k}{suffix}"
                if not candidate.exists():
                    dst = candidate
                    break
                k += 1

        shutil.move(str(src), str(dst))
        self.last_move = (src, dst)

        # удаляем из списка текущий файл и НЕ увеличиваем idx (потому что список сдвинулся)
        self.images.pop(self.idx)

        self.show_current()

    def mark_safe(self):
        self._move_current(self.safe_dir)

    def mark_danger(self):
        self._move_current(self.danger_dir)

    def skip(self):
        # просто перейти к следующему (файл остаётся на месте)
        self.idx += 1
        self.show_current()

    def undo(self):
        if not self.last_move:
            messagebox.showinfo("Undo", "Nothing to undo.")
            return

        src_original, dst_path = self.last_move
        # вернуть файл обратно в root_dir
        if not dst_path.exists():
            messagebox.showerror("Undo", f"Cannot find moved file:\n{dst_path}")
            return

        restored = self.root_dir / dst_path.name
        if restored.exists():
            # не затираем
            stem = restored.stem
            suffix = restored.suffix
            k = 1
            while True:
                candidate = self.root_dir / f"{stem}_restored_{k}{suffix}"
                if not candidate.exists():
                    restored = candidate
                    break
                k += 1

        shutil.move(str(dst_path), str(restored))

        # вернуть в список изображений на текущую позицию
        self.images.insert(self.idx, restored)
        self.last_move = None

        self.show_current()

    def run(self):
        self.window.mainloop()


def main():
        # Папка с неразмеченными изображениями: аргумент или текущая директория
        if len(sys.argv) >= 2:
            root_dir = Path(sys.argv[1])
        else:
            root_dir = Path.cwd()

        app = AnnotatorGUI(root_dir)
        app.run()


if __name__ == "__main__":
    main()
