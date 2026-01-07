import json

def compress_json(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as f:
        data = json.load(f)

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('[\n')
        for i, item in enumerate(data):
            json.dump(item, f, separators=(',', ':'))
            if i < len(data) - 1:
                f.write(',\n')
        f.write('\n]')

if __name__ == "__main__":
    input_file = 'map.json'
    output_file = 'compressed_map.json'
    compress_json(input_file, output_file)
