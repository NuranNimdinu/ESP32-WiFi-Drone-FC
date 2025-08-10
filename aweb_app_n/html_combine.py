# combine_files.py
import os

def combine_files(html_file, css_file, js_file, output_file):
    # Read CSS
    with open(css_file, 'r', encoding='utf-8') as f:
        css_content = f.read()

    # Read JS
    with open(js_file, 'r', encoding='utf-8') as f:
        js_content = f.read()

    # Read HTML and insert CSS & JS
    with open(html_file, 'r', encoding='utf-8') as f:
        html_content = f.read()

    # Replace <link rel="stylesheet" href="style.css"> with inline CSS
    html_content = html_content.replace(
        '<link rel="stylesheet" href="style.css">',
        f'<style>\n{css_content}\n</style>'
    )

    # Replace <script src="script.js"></script> with inline JS
    html_content = html_content.replace(
        '<script src="main.js"></script>',
        f'<script>\n{js_content}\n</script>'
    )

    # Save combined HTML
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(html_content)

    print(f"Combined HTML created as '{output_file}'")


# Example usage:
# file_path = input("Enter main html file path : ")
directory = os.path.abspath(__file__)
directory = os.path.dirname(directory)
print(directory)

combine_files(directory + '\index.html', directory + '\style.css', directory + '\main.js', directory + '\webpage.html')
