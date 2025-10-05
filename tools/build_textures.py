# this requires Wand and ImageMagick to be installed
# - create a virtual environment and activate it
# - pip-install wand: "pip install Wand"
# - follow these instructions: https://docs.wand-py.org/en/latest/guide/install.html#install-imagemagick-on-windows
#
# I you change the texture file, you MUST restart the game. Reloading the savegame is not sufficient.

WIDTH, HEIGHT = 2048, 2048

import dataclasses
import glob
import os
import wand.color
import wand.image


@dataclasses.dataclass
class ImageInfo:
    width: int
    height: int
    name: str
    path: str
    img: wand.image.Image
    slice: str = ""


def _find_spot(lines, width, image: ImageInfo):
    gap = 4
    w, h = image.width + gap, image.height + gap
    for y in range(len(lines) - h):
        if all(lines[y + i] + w <= width for i in range(h)):
            x = max(lines[y : y + h])
            for i in range(h):
                lines[y + i] = x + w
            return x, y
    return None


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    icon_src = os.path.join(script_dir, "icons")
    dst = os.path.join(script_dir, "..", "textures")
    output_image = os.path.join(dst, "ad_gui.dds")
    output_xml = os.path.join(dst, "ad_gui.xml")

    images: list[ImageInfo] = []
    names: set[str] = set()

    for image in glob.glob(os.path.join(icon_src, "*")):
        img = wand.image.Image(filename=image, background=wand.color.Color('transparent'))
        name, _ = os.path.splitext(os.path.basename(image))
        if name in names:
            raise ValueError(f"Duplicate image name: {name}")
        images.append(ImageInfo(img.width, img.height, name, image, img))
        names.add(name)
    images.sort(key=lambda x: x.height, reverse=True)

    lines = [0] * HEIGHT
    merged = wand.image.Image(width=WIDTH, height=HEIGHT, background=wand.color.Color("rgba(0, 0, 0, 0.0"), colorspace="srgb")
    merged.alpha_channel = "transparent"

    for image in images:
        pos = _find_spot(lines, WIDTH, image)
        if pos is None:
            raise ValueError(f"Failed to place {image.name}. Increase texture size.")
        x, y = pos
        merged.composite(image.img, left=x, top=y, operator="replace")
        image.slice = f'<slice id="{image.name}" uvs="{x}px {y}px {image.width}px {image.height}px"/>'
        print(f"Placed {image.name} at {x}, {y}")

    xml = """<texture>
    <meta>
        <filename>{texture_name}</filename>
        <size width="{width}" height="{height}"/>
    </meta>
    <slices>
{slices}
    </slices>
</texture>
"""
    with open(output_xml, "w") as f:
        f.write(
            xml.format(
                texture_name=os.path.basename(output_image.replace(".dds", ".png")),
                width=WIDTH,
                height=HEIGHT,
                slices="\n".join(f"        {x.slice}" for x in images),
            )
        )

    merged.compression = "dxt5"
    merged.options["dds:mipmaps"] = "0"
    merged.save(filename=output_image)
    merged.close()


if __name__ == "__main__":
    main()
