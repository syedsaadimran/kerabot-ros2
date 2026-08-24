import os
import re
import glob

def clean_file(fpath):
    with open(fpath, 'r', encoding='utf-8') as f:
        content = f.read()
    cleaned = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)
    cleaned = '\n'.join([line for line in cleaned.splitlines() if line.strip()])
    with open(fpath, 'w', encoding='utf-8') as f:
        f.write(cleaned + '\n')
    print(f"Cleaned: {fpath}")

def main():
    base_dir = '/home/saad/kerabot_ws/src/Robot_to_URDF_New_Pakka_description/urdf'
    for root, dirs, files in os.walk(base_dir):
        for file in files:
            if file.endswith(('.xacro', '.urdf')):
                clean_file(os.path.join(root, file))

if __name__ == '__main__':
    main()
