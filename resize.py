from PIL import Image

def resize_image(input_image_path, output_image_path, width=2365, height=1524):
    with Image.open(input_image_path) as img:
        # Resize the image using LANCZOS for high-quality resampling
        resized_img = img.resize((width, height), Image.Resampling.LANCZOS)
        # Save the resized image
        resized_img.save(output_image_path)
        print(f"Image saved to {output_image_path} with size {width}x{height}")

# Example usage
input_image_path = './map20000.png'  # Replace with your image path
output_image_path = 'map20000resized.png'  # Replace with the desired output path

resize_image(input_image_path, output_image_path)
