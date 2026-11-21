"""Original candy and printed wrapper appearance, separate from their physics."""
from pathlib import Path

import numpy as np


def wrapper_print(path):
    """Draw an unbranded wrapper in flat material coordinates."""
    from PIL import Image, ImageDraw

    width, height = 1024, 512
    image = Image.new('RGB', (width, height), '#b11654')
    draw = ImageDraw.Draw(image)
    # Both ends are cream with registration stripes that follow the twist.
    for left, right in ((0, 245), (779, 1024)):
        draw.rectangle((left, 0, right, height), fill='#f4e6c8')
        for y in range(-height, height, 48):
            draw.line((left, y + 42, right, y), fill='#c83467', width=11)
    for x in (242, 253, 766, 777):
        draw.line((x, 0, x, height), fill='#efbb6b', width=5)
    # Repeat the berry medallion around the wrapper, including the back.
    for y in (78, 250, 422):
        draw.rounded_rectangle((363, y - 58, 660, y + 58), radius=51, fill='#f8e8ce')
        for offset_x, offset_y, radius in ((-25, -8, 17), (0, -15, 19), (24, -5, 15),
                                             (-14, 13, 16), (13, 13, 15), (0, 26, 12)):
            x, center = 512 + offset_x, y + offset_y
            draw.ellipse((x - radius, center - radius, x + radius, center + radius), fill='#bc2b60')
            draw.ellipse((x - 7, center - 8, x - 2, center - 3), fill='#f384a0')
        draw.polygon([(508, y - 25), (470, y - 36), (493, y - 49)], fill='#398164')
        draw.polygon([(512, y - 27), (533, y - 48), (549, y - 34)], fill='#398164')
    draw.line((0, 125, width, 125), fill='#273052', width=7)
    image.save(path)


def _material(stage, path, color, roughness, coat):
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + '/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput('clearcoat', Sdf.ValueTypeNames.Float).Set(coat)
    shader.CreateInput('clearcoatRoughness', Sdf.ValueTypeNames.Float).Set(.22)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return material, shader


def decorate_wrapper(mesh, path):
    from pxr import Sdf, UsdGeom, UsdShade, Vt

    uv = [(along, across) for along in np.linspace(0., 1., 65)
          for across in np.linspace(0., 1., 73)]
    assert len(uv) == len(mesh.GetPointsAttr().Get())
    array = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        'st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    array.Set(Vt.Vec2fArray(uv))
    texture_file = Path(path).with_name('wrapper-print.png')
    wrapper_print(texture_file)
    stage = mesh.GetPrim().GetStage()
    material, surface = _material(stage, '/Asset/Print', (.60, .045, .15), .32, .55)
    reader = UsdShade.Shader.Define(stage, '/Asset/Print/Coordinates')
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname', Sdf.ValueTypeNames.String).Set('st')
    reader.CreateOutput('result', Sdf.ValueTypeNames.Float2)
    texture = UsdShade.Shader.Define(stage, '/Asset/Print/Ink')
    texture.CreateIdAttr('UsdUVTexture')
    texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('./' + texture_file.name))
    texture.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
    texture.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
    texture.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
    surface.GetInput('diffuseColor').ConnectToSource(texture.ConnectableAPI(), 'rgb')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def glaze_sweet(path):
    """Render a continuous rounded bevel inside the original collision hull."""
    from pxr import Usd, UsdGeom, UsdShade, Vt

    stage = Usd.Stage.Open(str(path))
    hull = UsdGeom.Mesh.Get(stage, '/Asset/hard_drop')
    contact_rings = np.asarray(hull.GetPointsAttr().Get()).reshape(6, 12, 3)
    heights = contact_rings[:, 0, 2]
    radii = contact_rings[:, 0, 0] * np.cos(np.pi / 12)
    sections = np.sort(np.r_[heights, (heights[:-1]+heights[1:])/2])
    r = np.interp(sections, heights, radii)
    angles = np.arange(72) * 2 * np.pi / 72
    points = [(radius*np.cos(a), radius*np.sin(a), z) for radius,z in zip(r,sections)
              for a in angles]
    n = len(angles)
    faces = [list(reversed(range(n)))]
    for i in range(len(sections)-1):
        for j in range(n):
            a, b = i*n+j, i*n+(j+1)%n
            faces.append([a,b,b+n,a+n])
    faces.append(list(range((len(sections)-1)*n, len(sections)*n)))
    mesh = UsdGeom.Mesh.Define(stage, '/Asset/glazed_sweet')
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(points, dtype=np.float32)))
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    mesh.CreateSubdivisionSchemeAttr('catmullClark')
    material, _ = _material(stage, '/Asset/Sugar', (.62, .038, .12), .24, .78)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    hull.MakeInvisible()
    stage.GetRootLayer().Save()
