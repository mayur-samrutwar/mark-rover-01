"""Native driver fixtures exercise the real subprocess path without Apple/GPU use."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import native
import pipeline

FAKE_NATIVE = r'''
import json,pathlib,sys
from PIL import Image
args=sys.argv[1:];cwd=pathlib.Path.cwd()
with (cwd/'native-calls.jsonl').open('a') as f:f.write(json.dumps(args)+'\n')
if '--probe' in args:
 print(json.dumps({'event':'capability','supported':True,'quality_profile_arguments':True,'maximumInputImageDimension':16384,'maximumNumberOfInputImages':1000}));sys.exit(0)
def get(flag):return args[args.index(flag)+1]
assert '--allow-process' in args and get('--formats')=='obj'
assert get('--ordering') in ('unordered','sequential') and get('--sensitivity') in ('normal','high')
out=pathlib.Path(get('--output'));detail=get('--detail');folder=out/('obj-'+detail);folder.mkdir(parents=True)
Image.new('RGB',(2,2),(160,120,80)).save(folder/'texture.png')
(folder/'scene.mtl').write_text('newmtl photo\nmap_Kd texture.png\n')
(folder/'scene.obj').write_text('mtllib scene.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nvt 0 0\nvt 1 0\nvt 0 1\nvt 1 1\nusemtl photo\nf 1/1 3/3 2/2\nf 1/1 2/2 4/4\nf 2/2 3/3 4/4\nf 3/3 1/1 4/4\n')
(folder/'scene.usda').write_text('#usda1.0\n(\nupAxis = "Y"\nmetersPerUnit = 1\ncreator = "fake test engine"\n)\n')
poses=[]
for index,p in enumerate(sorted(pathlib.Path(get('--input')).iterdir())):
 poses.append({'sample_id':index,'input_url':str(p),'translation':[index,1,3],
  'native_transform_rows':[[1,0,0,index],[0,1,0,1],[0,0,1,3],[0,0,0,1]]})
(out/'poses-native.json').write_text(json.dumps({'poses':poses}))
(out/'pointcloud-native.ply').write_text('ply\nformat ascii 1.0\nelement vertex3\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 0 0\n0 1 0\n')
(out/'bounds.json').write_text(json.dumps({'minimum':[0,0,0],'maximum':[1,1,1]}))
(out/'session-config.json').write_text(json.dumps({'detail':detail,'formats':'obj'}))
(out/'result.json').write_text(json.dumps({'status':'complete','elapsed_seconds':.01,'automatic_downsampling':False,
 'invalid_samples':[],'skipped_sample_ids':[],'stitching_incomplete':False,'request_errors':[]}))
print(json.dumps({'event':'requestProgress','fraction':1}))
'''


@unittest.skipUnless(importlib.util.find_spec('trimesh'), 'reconstruction dependencies unavailable')
class NativeTests(unittest.TestCase):
    def setUp(self):
        from PIL import Image
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        (self.root/'images').mkdir()
        for name,color in [('a.jpg',(10,20,30)),('b.jpg',(40,50,60)),('c.jpg',(70,80,90))]:
            exif=Image.Exif();exif[274]=6
            Image.new('RGB',(40,30),color).save(self.root/'images'/name,exif=exif)
        (self.root/'source-plan.json').write_text(json.dumps({'images':[{'filename':n} for n in ['b.jpg','a.jpg','c.jpg']]}))
        self.helper=self.root/'fake-native'
        self.helper.write_text('#!'+sys.executable+'\n'+FAKE_NATIVE)
        self.helper.chmod(0o755)

    def tearDown(self):
        self.temp.cleanup()

    def prepare(self):
        paths=native._ordered_inputs(self.root)
        hashes={p.name:pipeline._hash(p) for p in paths}
        return native._prepare_inputs(self.root,paths,hashes,threading.Event())

    def test_numbered_snapshot_keeps_exif_bytes_order_and_reuses_existing_copy(self):
        from PIL import Image
        manifest=self.prepare()
        self.assertEqual([x['filename'] for x in manifest['images']],['b.jpg','a.jpg','c.jpg'])
        first=self.root/'native/input/000001.jpg'
        before=first.stat().st_mtime_ns
        self.assertEqual(first.read_bytes(),(self.root/'images/b.jpg').read_bytes())
        with Image.open(first) as image:self.assertEqual(image.getexif()[274],6)
        self.prepare()
        self.assertEqual(first.stat().st_mtime_ns,before)
        stop=threading.Event();stop.set()
        with self.assertRaises(pipeline.PipelineCancelled):
            native._prepare_inputs(self.root,native._ordered_inputs(self.root),{},stop)

    def test_exif_camera_axes_are_proper_and_forward_projection_is_consistent(self):
        import numpy as np
        for orientation in (1,3,6,8):
            axes=native.upright_camera_axes(orientation)
            np.testing.assert_allclose(axes.T@axes,np.eye(3))
            self.assertAlmostEqual(np.linalg.det(axes),1)
            np.testing.assert_allclose(axes@np.array([0,0,1]),[0,0,-1])
        np.testing.assert_allclose(native.upright_camera_axes(6),[[0,1,0],[1,0,0],[0,0,-1]])
        self.assertIsNone(native.upright_camera_axes(2))
        manifest=self.prepare()
        poses={'poses':[{'sample_id':0,'input_url':str(self.root/'native/input/000001.jpg'),
             'native_transform_rows':[[1,0,0,2],[0,1,0,3],[0,0,1,4],[0,0,0,1]]}]}
        p=self.root/'poses.json';p.write_text(json.dumps(poses))
        camera=native.export_cameras(p,manifest)['cameras'][0]
        R=np.array(camera['rotation_world_to_camera']);C=np.array(camera['center']);t=np.array(camera['translation'])
        np.testing.assert_allclose(R@C+t,[0,0,0])
        np.testing.assert_allclose(R@(C+[0,0,-1])+t,[0,0,1])
        self.assertFalse(camera['intrinsics_known'])
        self.assertEqual(camera['preview_fov_degrees'],60)
        self.assertEqual(native.export_cameras(p,manifest)['camera_models'],{})

    def test_probe_is_explicit_and_does_not_process_images(self):
        capability=native.probe_native(self.helper,self.root)
        self.assertTrue(capability['supported'])
        calls=[json.loads(x) for x in (self.root/'native-calls.jsonl').read_text().splitlines()]
        self.assertEqual(calls,[['--probe']])
        self.assertFalse((self.root/'native/output').exists())

    @unittest.skipUnless(hasattr(os,'killpg'),'POSIXprocessgroups required')
    def test_cancelled_probe_terminates_the_process_group_promptly(self):
        helper=self.root/'slow-native'
        helper.write_text('#!'+sys.executable+'\nimport time\ntime.sleep(30)\n');helper.chmod(0o755)
        stop=threading.Event();timer=threading.Timer(.1,stop.set);timer.start()
        started=time.monotonic()
        try:
            with self.assertRaises(pipeline.PipelineCancelled):native.probe_native(helper,self.root,stop)
        finally:timer.cancel()
        self.assertLess(time.monotonic()-started,3)

    def test_completed_native_output_is_reused_and_exports_remain_in_native_frame(self):
        original={p.name:p.read_bytes() for p in (self.root/'images').iterdir()}
        updates=[];config={'quality':'full','native_engine_path':str(self.helper),'max_threads':1,'stage_timeout_seconds':20}
        first=native.run_native(self.root,config,updates.append,threading.Event())
        for artifact in first['artifacts']:
            self.assertEqual(artifact['bytes'],(self.root/artifact['path']).stat().st_size,artifact['path'])
        second=native.run_native(self.root,config,updates.append,threading.Event())
        for artifact in second['artifacts']:
            self.assertEqual(artifact['bytes'],(self.root/artifact['path']).stat().st_size,artifact['path'])
        persisted=json.loads((self.root/'exports/pipeline-result.json').read_text())
        self.assertNotIn('exports/pipeline-result.json',[x['path'] for x in persisted['artifacts']])
        self.assertIn('exports/pipeline-result.json',[x['path'] for x in second['artifacts']])
        with zipfile.ZipFile(self.root/'exports/scene.zip') as archive:
            bundled=json.loads(archive.read('manifest.json'))
            geometry_artifact=next(x for x in bundled['artifacts'] if x['path']=='exports/geometry.json')
            self.assertEqual(geometry_artifact['bytes'],archive.getinfo('geometry.json').file_size)
        calls=[json.loads(x) for x in (self.root/'native-calls.jsonl').read_text().splitlines()]
        processing=[x for x in calls if '--probe' not in x]
        self.assertEqual(len(processing),1)
        self.assertEqual(processing[0][processing[0].index('--ordering')+1],'unordered')
        self.assertEqual(processing[0][processing[0].index('--sensitivity')+1],'normal')
        self.assertEqual(processing[0][processing[0].index('--detail')+1],'raw')
        self.assertEqual(processing[0][processing[0].index('--formats')+1],'obj')
        self.assertEqual(first['backend'],{'requested':'apple','active':'apple'})
        self.assertIn('Apple',second['metrics']['coordinate_frame'])
        self.assertFalse(second['metrics']['metric_scale_known'])
        self.assertFalse(second['metrics']['collision_validated'])
        self.assertEqual(second['metrics']['registered_images'],3)
        cameras=json.loads((self.root/'exports/cameras.json').read_text())
        self.assertEqual(cameras['up_axis'],'Y');self.assertEqual(cameras['camera_models'],{})
        self.assertEqual(updates[-1]['progress'],1)
        for name,data in original.items():self.assertEqual((self.root/'images'/name).read_bytes(),data)
        self.assertTrue((self.root/'exports/scene.glb').is_file())
        self.assertTrue((self.root/'exports/scene.zip').is_file())


    def test_profile_change_invalidates_cached_native_geometry(self):
        config={'quality':'full','native_engine_path':str(self.helper),'max_threads':1,'stage_timeout_seconds':20}
        native.run_native(self.root,config,lambda _:None,threading.Event())
        native.run_native(self.root,{**config,'native_ordering':'sequential','native_sensitivity':'high'},lambda _:None,threading.Event())
        calls=[json.loads(x) for x in (self.root/'native-calls.jsonl').read_text().splitlines()]
        processing=[x for x in calls if '--probe' not in x]
        self.assertEqual(len(processing),2)
        self.assertEqual(processing[1][processing[1].index('--ordering')+1],'sequential')
        state=json.loads((self.root/'native-state.json').read_text())
        self.assertEqual(state['configuration']['configuration']['featureSensitivity'],'high')

    def test_old_helper_is_rejected_before_silent_profile_mismatch(self):
        old=self.helper.read_text().replace("'quality_profile_arguments':True,",'')
        self.helper.write_text(old)
        config={'quality':'full','native_engine_path':str(self.helper),'max_threads':1,'stage_timeout_seconds':20}
        with self.assertRaisesRegex(pipeline.PipelineError,'predates'):
            native.run_native(self.root,config,lambda _:None,threading.Event())
        calls=[json.loads(x) for x in (self.root/'native-calls.jsonl').read_text().splitlines()]
        self.assertEqual(calls,[['--probe']])

    def test_invalid_native_profile_fails_before_starting_engine(self):
        with self.assertRaises(pipeline.PipelineError):
            native.run_native(self.root,{'native_ordering':'unknown'},lambda _:None,threading.Event())
        self.assertFalse((self.root/'native-calls.jsonl').exists())


if __name__=='__main__':unittest.main()
