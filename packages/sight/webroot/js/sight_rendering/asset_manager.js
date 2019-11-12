/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Asset manager
// Supports:
//  - STL file (mesh.stl = '...')
//  - OBJ file (mesh.obj = '...')
//    - Diffuse + Normal map PNG provided (mesh.diffuse_map + mesh.normal_map)
//    - MLT file (mesh.mlt = '...')
// Additional parameters are supported:
//  - rotation: quaternion [w, x, y, z]
//  - translation: Vector3 [x, y, z]
//  - scale: float
//  - pixel_size: size of a pixel for the 2D image
//  - material:
//    - color: integer [0, 16777215]
//    - specular: integer [0, 16777215]
//    - shininess: integer [0, 255]
class AssetManagerImpl {
  constructor() {
    this.assets_ = {}
    this.cached_ = {}
  }

  // Load a list of assets from a config
  loadAssets(config) {
    if (config === null || !("assets" in config)) {
      return;
    }
    for (let name in config.assets) {
      let asset = config.assets[name];
      this.assets_[name] = this.load3DAsset_(name, asset);
    }
  }

  // Returns an asset.
  getAsset3D(name, full_channel) {
    if (!(full_channel in this.cached_)) {
      this.cached_[full_channel] = {};
    }
    if (!(name in this.cached_[full_channel])) {
      if (!(name in this.assets_) || this.assets_[name].is_loaded !== true) {
        return null;
      }
      // Make a copy for a specific full_channel
      let clone = this.assets_[name].clone(true);
      this.cached_[full_channel][name] = clone;
      clone.shared = true;
      // Find the material:
      let node = clone;
      while (!node.material && node.children && node.children.length > 0) {
        node = node.children[0];
      }
      // Make a copy of the material to allow SopStyle to change the color
      if (node.material) {
        clone.default_material = node.material.clone();
        clone.backup_material = clone.default_material.clone();
        clone.traverse(function(child) { child.material = clone.default_material; });
      }
    }
    return this.cached_[full_channel][name];
  }

  // Returns an asset.
  getAsset2D(name) {
    if (!(name in this.assets_)) {
      return null;
    }
    return this.assets_[name];
  }

  // Delete a specific asset
  removeAsset(name) {
    for (let renderer in this.cached_) {
      if (name in this.cached_[renderer]) {
        this.deleteOperation_(this.cached_[renderer][name]);
        delete this.cached_[renderer][name];
      }
    }
    if (name in this.assets_) {
      this.deleteOperation_(this.assets_[name]);
      delete this.assets_[name];
    }
  }

  // Remove all the assets
  removeAllAssets() {
    let list = [];
    for (let name in this.assets_) {
      list.push(name);
    }
    for (let name in list) {
      this.removeAsset(name);
    }
    this.assets_ = {};
    this.cached_ = {};
  }


  // Remove the operation from the scene if present, and destruct it
  deleteOperation_(obj) {
    if (obj === null) return;
    if (obj.children !== undefined) {
      while (obj.children.length > 0) {
        this.deleteOperation_(obj.children[0]);
        obj.remove(obj.children[0]);
      }
    }
    if (obj.geometry) {
      obj.geometry.dispose();
      delete obj.geometry;
    }
    if (obj.material) {
      obj.material.dispose();
      delete obj.material;
    }
    if (obj.dispose instanceof Function) {
      obj.dispose();
    }
  }

  // Creates a 3JS object to render a mesh
  load3DAsset_(name, mesh) {
    let asset = new THREE.Group();
    asset.is_loaded = false;
    asset.shared = mesh.unique;

    // Default material with textures
    let material_config = mesh.material || {};
    if (material_config.color === undefined) {
      material_config.color =  0x7f7f7f;
    }
    if (material_config.specular === undefined) {
      material_config.specular =  0x111111;
    }
    if (material_config.shininess === undefined) {
      material_config.shininess =  200;
    }
    let material = new THREE.MeshPhongMaterial(material_config);

    const that = this;

    // To be called once the object is loaded, if material is defined, it will set for every child.
    let postLoading = function(object, material) {
      // Set material and shadow
      let applyMaterial = function(child) {
        if (material && material !== null) {
          child.material = material;
        }
        child.castShadow = true;
        child.receiveShadow = true;
      };
      object.traverse(applyMaterial);

      // Apply various transformation if needed
      if (mesh.rotation) {
        // Order: (X, Y, Z, W)
        object.quaternion.set(
            mesh.rotation[1], mesh.rotation[2], mesh.rotation[3], mesh.rotation[0]);
      }
      if (mesh.translation) {
        object.position.set(
            mesh.translation[0], mesh.translation[1], mesh.translation[2]);
      }
      if (mesh.scale) {
        object.scale.set(mesh.scale, mesh.scale, mesh.scale);
      }

      asset.add(object);
      // Build the 2D object
      const kDefaultPixelSize = 0.05;
      asset.img = that.load2DAsset_(asset, mesh.pixel_size || kDefaultPixelSize);
      asset.is_loaded = true;
      console.log(name + ': fully loaded');
    }

    const loadingUpdate = function(xhr) {
      console.log(name + ':' + xhr.loaded + '/' + xhr.total);
    }
    const loadingError = function(error) {
      console.log('An error happened while loading ' + name);
      console.error(error);
    }

    // Load mtl + obj file
    if (mesh.mtl) {
      new THREE.MTLLoader().load(mesh.mtl, function(mat) {
          mat.preload();
          let objloader = new THREE.OBJLoader();
          objloader.setMaterials(mat);
          objloader.load(mesh.obj, postLoading, loadingUpdate, loadingError);
      });
      return asset;
    }

    // If a diffuse_map and normal_map are provided, load the custom material
    if (mesh.diffuse_map && mesh.normal_map) {
      let texloader = new THREE.TextureLoader();
      material = new THREE.MeshLambertMaterial();
      material.map = texloader.load(mesh.diffuse_map);
      material.normalMap = texloader.load(mesh.normal_map);
    }

    // Load an obj file with the current default texture
    if (mesh.obj) {
      // mesh from obj
      new THREE.OBJLoader().load(mesh.obj, function(object) { postLoading(object, material); },
                                 loadingUpdate, loadingError);
      return asset;
    }
    // Load an stl file
    if (mesh.stl) {
      // mesh from obj
      new THREE.STLLoader().load(
          mesh.stl, function(geometry) {
            let mesh = new THREE.Mesh(geometry, material);
            postLoading(mesh, material);
          },
          loadingUpdate, loadingError);
    }
    return asset;
  }

  // Creates a 2D canvas containing an orthographic projection of a 3JS object.
  load2DAsset_(object_3d, pixel_size) {
    // Computes the bounding box of the object recursively
    let extractBoundingBox = function(obj) {
      if (obj.geometry) {
        obj.geometry.computeBoundingBox();
        return obj.geometry.boundingBox.clone().applyMatrix4(obj.matrixWorld.clone());
      }

      if (obj.children && obj.children.length > 0) {
        let box = extractBoundingBox(obj.children[0]);
        for (let i = 1; i < obj.children.length; i++) {
          box.union(extractBoundingBox(obj.children[i]));
        }
        return box;
      }
      return null;
    }
    // 3D Canvas the object is going to be rendered
    let canvas = document.createElement("canvas");
    let canvas_2d = document.createElement("canvas");
    // Force compute the matrix world for the object to transform the bounding box
    object_3d.updateMatrixWorld(true);
    const box = extractBoundingBox(object_3d);
    if (box === null) return;
    // Add some margin and create an image center on the object
    canvas.width = 4 + 2 * (Math.max(-box.min.y, box.max.y) / pixel_size) | 0;
    canvas.height = 4 + 2 * (Math.max(-box.min.x, box.max.x) / pixel_size) | 0;

    // Create a renderer
    let renderer = new THREE.WebGLRenderer({
        alpha: true,
        canvas: canvas,
        preserveDrawingBuffer: true
    });

    // Create a scene with proper lighting
    let scene = new THREE.Scene();
    renderer.setClearColor(0xffffff, 0);  // Transperent background
    // Set the orthographic camera and position in on top of the object
    const camera = new THREE.OrthographicCamera(
        -canvas.width * 0.5 * pixel_size, canvas.width * 0.5 * pixel_size,
        canvas.height * 0.5 * pixel_size, -canvas.height * 0.5 * pixel_size,
        0.1, 1000);
    camera.position.set(0.0, 0.0, 500.0);
    camera.lookAt(0.0, 0.0, 0.0);
    camera.rotateZ(Math.PI/2);
    camera.updateProjectionMatrix();

    // create env lights
    let ambient = new THREE.AmbientLight(0x7f7f7f);
    scene.add(ambient);
    let directionalLight = new THREE.DirectionalLight(0xffffff);
    directionalLight.position.set(0, 500, 500);
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 2048;
    directionalLight.shadow.mapSize.height = 2048
    directionalLight.shadow.camera.near = 1;
    directionalLight.shadow.camera.far = 5000;
    directionalLight.shadow.soft = true;
    scene.add(directionalLight);

    // Add the object
    scene.add(object_3d);

    // Render the scene
    renderer.render(scene, camera);

    // Set the information about the canvas needed for the 2D rendering
    canvas_2d.width = canvas.width;
    canvas_2d.height = canvas.height;
    canvas_2d.getContext("2d").drawImage(canvas, 0, 0);
    canvas_2d.scale = pixel_size;
    canvas_2d.center = true;
    renderer.forceContextLoss();
    renderer.dispose();
    return canvas_2d;
  }
}

let asset_manager_ = null;
// Implement a "singleton" of AssetManager
function AssetManager() {
  if (asset_manager_ == null) {
    asset_manager_ = new AssetManagerImpl();
  }
  return asset_manager_;
}