"""Exercise verified transfers with a private localhost HTTPS fixture."""

import hashlib
import http.server
import multiprocessing
import socket
import ssl
import threading
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from pathlib import Path
from xml.etree import ElementTree

import dartpy as dart
import pytest

pytestmark = pytest.mark.skipif(
    not hasattr(dart.utils, "ModelResourceRetriever"),
    reason="dartpy built without utils-assets",
)

SKEL_SKELETON = b"""<skeleton name="testbot"><body name="base">
  <inertia><mass>1</mass><moment_of_inertia>
    <ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
  </moment_of_inertia></inertia>
  <visualization_shape><geometry><mesh>
    <file_name>meshes/link.obj</file_name><scale>1 1 1</scale>
  </mesh></geometry></visualization_shape></body>
  <joint name="root" type="weld"><parent>world</parent><child>base</child></joint>
</skeleton>"""
SDF_MODEL = b"""<model name="testbot"><link name="base">
  <inertial><mass>1</mass><inertia>
    <ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
  </inertia></inertial>
  <visual name="box"><geometry><box><size>1 1 1</size></box></geometry></visual>
</link></model>"""
FILES = {
    "robot.urdf": b'<robot name="testbot"><link name="base"/></robot>',
    "robot.skel": b'<skel version="1.0">'
    + SKEL_SKELETON
    + b'<world name="testworld">'
    + SKEL_SKELETON
    + b"</world></skel>",
    "robot.sdf": b'<sdf version="1.6">'
    + SDF_MODEL
    + b'<world name="testworld">'
    + SDF_MODEL
    + b"</world></sdf>",
    "robot.xml": b"""<mujoco model="testbot"><worldbody><body name="base">
      <geom name="box" type="box" size="0.5 0.5 0.5"/>
    </body></worldbody></mujoco>""",
    "meshes/link.obj": b"mtllib link.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
    "meshes/link.mtl": b"newmtl body\nmap_Kd textures/body.png\n",
    "meshes/textures/body.png": b"test-only-texture",
}
MODEL_URI = "model://testbot/v1/robot.urdf"


@contextmanager
def local_https_server(before_response=None):
    responses = dict(FILES)
    requests = []
    failures = set()
    interrupted = set()

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.removeprefix("/")
            requests.append(path)
            if path in failures or path not in responses:
                self.send_error(503)
                return
            if before_response:
                before_response()
            body = responses[path]
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            if path in interrupted:
                self.wfile.write(body[: max(1, len(body) // 2)])
                self.wfile.flush()
                self.connection.shutdown(socket.SHUT_RDWR)
                return
            self.wfile.write(body)

        def log_message(self, *_args):
            pass

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    certificate = Path(__file__).with_name("fixtures") / "localhost-test.crt"
    key = certificate.with_suffix(".key")
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    # The committed private key is solely for this isolated test server.
    context.load_cert_chain(certificate, key)
    server.socket = context.wrap_socket(server.socket, server_side=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    fixture = {
        "certificate": certificate,
        "url": f"https://localhost:{server.server_port}",
        "responses": responses,
        "requests": requests,
        "failures": failures,
        "interrupted": interrupted,
    }
    try:
        yield fixture
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def trust_test_server(monkeypatch, server):
    monkeypatch.setenv("CURL_CA_BUNDLE", str(server["certificate"]))
    monkeypatch.setenv("NO_PROXY", "localhost,127.0.0.1")
    monkeypatch.setenv("no_proxy", "localhost,127.0.0.1")


@pytest.fixture
def https_server(monkeypatch):
    with local_https_server() as server:
        trust_test_server(monkeypatch, server)
        yield server


def write_manifest(directory, url, revision="v1", override=None):
    model = ElementTree.Element(
        "model",
        schemaVersion="1",
        id="testbot",
        revision=revision,
        entrypoint="robot.urdf",
    )
    ElementTree.SubElement(model, "source", url=f"{url}/pinned-source")
    ElementTree.SubElement(model, "license", name="MIT")
    ElementTree.SubElement(model, "package", name="testbot_description", path=".")
    for path, body in FILES.items():
        attributes = {
            "path": path,
            "url": f"{url}/{path}",
            "sha256": hashlib.sha256(body).hexdigest(),
            "size": str(len(body)),
        }
        if override and path == override[0]:
            attributes.update(override[1])
        ElementTree.SubElement(model, "file", **attributes)
    manifest = directory / f"manifest-{revision}.xml"
    manifest.write_bytes(ElementTree.tostring(model))
    return manifest


def make_retriever(cache, manifest, offline=False):
    retriever = dart.utils.ModelResourceRetriever(str(cache), offline)
    assert retriever.addManifest(manifest.as_uri())
    return retriever


def bundle_path(cache, manifest, revision="v1"):
    return (
        cache / "testbot" / revision / hashlib.sha256(manifest.read_bytes()).hexdigest()
    )


def cold_access_worker(connection, directory, operation):
    started = threading.Event()
    response_allowed = threading.Event()

    def before_response():
        started.set()
        assert response_allowed.wait(timeout=15)

    try:
        with local_https_server(
            before_response if operation == "register" else None
        ) as server, pytest.MonkeyPatch.context() as monkeypatch:
            trust_test_server(monkeypatch, server)
            directory = Path(directory)
            manifest = write_manifest(directory, server["url"])
            cache = directory / "cache"
            retriever = make_retriever(cache, manifest)
            connection.send("ready")
            if operation == "package":
                packages = dart.utils.PackageResourceRetriever(retriever)
                packages.addPackageDirectory(
                    "testbot_description", "model://testbot/v1"
                )
                path = packages.getFilePath("package://testbot_description/robot.urdf")
                assert Path(path) == bundle_path(cache, manifest) / "robot.urdf"
            elif operation == "register":
                with ThreadPoolExecutor(max_workers=1) as executor:
                    acquisition = executor.submit(retriever.exists, MODEL_URI)
                    assert started.wait(timeout=5)
                    resume = threading.Timer(0.05, response_allowed.set)
                    resume.start()
                    assert retriever.addManifest(manifest.as_uri())
                    assert acquisition.result(timeout=5)
                    resume.join(timeout=5)
            elif operation.startswith("skel_"):
                uri = "model://testbot/v1/robot.skel"
                if operation == "skel_skeleton":
                    robot = dart.utils.SkelParser.readSkeleton(uri, retriever)
                else:
                    if operation == "skel_xml":
                        world = dart.utils.SkelParser.readWorldXML(
                            FILES["robot.skel"].decode(), uri, retriever
                        )
                    else:
                        world = dart.utils.SkelParser.readWorld(uri, retriever)
                    assert world is not None
                    assert world.getNumSkeletons() == 1
                    robot = world.getSkeleton(0)
                assert robot is not None
                assert robot.getNumBodyNodes() == 1
                assert robot.getBodyNode(0).getNumShapeNodes() == 1
            elif operation.startswith("sdf_"):
                uri = "model://testbot/v1/robot.sdf"
                options = (
                    retriever
                    if operation.endswith("legacy")
                    else dart.utils.SdfParser.Options(retriever)
                )
                if operation.startswith("sdf_world"):
                    world = dart.utils.SdfParser.readWorld(uri, options)
                    assert world is not None
                    assert world.getNumSkeletons() == 1
                    robot = world.getSkeleton(0)
                else:
                    robot = dart.utils.SdfParser.readSkeleton(uri, options)
                assert robot is not None
                assert robot.getNumBodyNodes() == 1
            elif operation == "mjcf":
                world = dart.utils.MjcfParser.readWorld(
                    "model://testbot/v1/robot.xml",
                    dart.utils.MjcfParser.Options(retriever),
                )
                assert world is not None
                assert world.getNumSkeletons() == 1
                assert world.getSkeleton(0).getNumBodyNodes() == 1
            else:
                composite = dart.utils.CompositeResourceRetriever()
                composite.addSchemaRetriever("model", retriever)
                if operation == "composite":
                    path = composite.getFilePath(MODEL_URI)
                    assert Path(path) == bundle_path(cache, manifest) / "robot.urdf"
                else:
                    loader = dart.utils.DartLoader()
                    if operation == "legacy_loader":
                        robot = loader.parseSkeleton(MODEL_URI, composite)
                    else:
                        loader.setOptions(
                            dart.utils.DartLoaderOptions(
                                composite, dart.utils.DartLoaderRootJointType.FIXED
                            )
                        )
                        robot = loader.parseSkeleton(MODEL_URI)
                    assert robot is not None
                    assert robot.getNumBodyNodes() == 1
            assert sorted(server["requests"]) == sorted(FILES)
        connection.send(None)
    except Exception as error:
        connection.send(f"{type(error).__name__}: {error}")
    finally:
        connection.close()


@pytest.mark.parametrize(
    "operation",
    [
        "package",
        "composite",
        "loader",
        "legacy_loader",
        "register",
        "skel_world",
        "skel_xml",
        "skel_skeleton",
        "sdf_world",
        "sdf_world_legacy",
        "sdf_skeleton",
        "sdf_skeleton_legacy",
        "mjcf",
    ],
)
def test_cold_cpp_callers_release_gil_for_https(tmp_path, operation):
    # A separate process bounds failures when a C++ call prevents server threads running.
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe(duplex=False)
    worker = context.Process(
        target=cold_access_worker, args=(child, str(tmp_path), operation)
    )
    worker.start()
    child.close()
    try:
        assert parent.poll(30), "HTTPS test worker did not initialize"
        assert parent.recv() == "ready"
        worker.join(timeout=15)
        assert not worker.is_alive(), f"{operation} blocked Python HTTPS server threads"
        assert worker.exitcode == 0
        assert parent.poll(), "HTTPS test worker did not report its result"
        assert parent.recv() is None
    finally:
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=5)
        parent.close()


def test_cold_exists_fetches_complete_bundle_and_offline_reuses_it(
    tmp_path, https_server
):
    manifest = write_manifest(tmp_path, https_server["url"])
    cache = tmp_path / "cache"
    retriever = make_retriever(cache, manifest)

    assert retriever.exists(MODEL_URI)
    assert sorted(https_server["requests"]) == sorted(FILES)
    bundle = bundle_path(cache, manifest)
    for path, body in FILES.items():
        assert (bundle / path).read_bytes() == body
    assert Path(retriever.getFilePath(MODEL_URI)) == bundle / "robot.urdf"
    assert retriever.readAll(MODEL_URI) == FILES["robot.urdf"].decode()

    count = len(https_server["requests"])
    offline = make_retriever(cache, manifest, offline=True)
    assert offline.exists(MODEL_URI)
    assert offline.retrieve(MODEL_URI) is not None
    assert len(https_server["requests"]) == count


def test_tls_verification_rejects_untrusted_server(tmp_path, https_server, monkeypatch):
    manifest = write_manifest(tmp_path, https_server["url"])
    monkeypatch.delenv("CURL_CA_BUNDLE")
    monkeypatch.delenv("SSL_CERT_FILE", raising=False)
    cache = tmp_path / "cache"
    retriever = make_retriever(cache, manifest)

    assert not retriever.exists(MODEL_URI)
    assert not bundle_path(cache, manifest).exists()
    assert not https_server["requests"]


def test_tls_verification_checks_hostname(tmp_path, https_server):
    manifest = write_manifest(
        tmp_path, https_server["url"].replace("localhost", "127.0.0.1")
    )
    cache = tmp_path / "cache"
    retriever = make_retriever(cache, manifest)

    assert not retriever.exists(MODEL_URI)
    assert not bundle_path(cache, manifest).exists()
    assert not https_server["requests"]


def test_ssl_cert_file_supplies_trusted_ca_when_curl_bundle_is_unset(
    tmp_path, https_server, monkeypatch
):
    certificate = Path(__file__).with_name("fixtures") / "localhost-test.crt"
    monkeypatch.delenv("CURL_CA_BUNDLE")
    monkeypatch.setenv("SSL_CERT_FILE", str(certificate))
    manifest = write_manifest(tmp_path, https_server["url"])
    retriever = make_retriever(tmp_path / "cache", manifest)

    assert retriever.exists(MODEL_URI)
    assert sorted(https_server["requests"]) == sorted(FILES)


@pytest.mark.parametrize(
    "attributes",
    [{"sha256": "0" * 64}, {"size": "1"}, {"size": "999999"}],
)
def test_integrity_failure_never_publishes(tmp_path, https_server, attributes):
    manifest = write_manifest(
        tmp_path, https_server["url"], override=("robot.urdf", attributes)
    )
    cache = tmp_path / "cache"
    retriever = make_retriever(cache, manifest)

    assert not retriever.exists(MODEL_URI)
    assert not bundle_path(cache, manifest).exists()
    assert https_server["requests"]
    assert not list((cache / "testbot" / "v1").glob(".tmp-*"))


@pytest.mark.parametrize("failure", ["failures", "interrupted"])
def test_failed_transfer_is_not_reused_and_clean_retry_succeeds(
    tmp_path, https_server, failure
):
    manifest = write_manifest(tmp_path, https_server["url"])
    cache = tmp_path / "cache"
    https_server[failure].add("meshes/textures/body.png")
    retriever = make_retriever(cache, manifest)

    assert not retriever.exists(MODEL_URI)
    assert not bundle_path(cache, manifest).exists()
    offline = make_retriever(cache, manifest, offline=True)
    assert not offline.exists(MODEL_URI)
    https_server[failure].clear()
    assert retriever.exists(MODEL_URI)
    for path, body in FILES.items():
        assert (bundle_path(cache, manifest) / path).read_bytes() == body


def test_concurrent_downloaders_publish_one_complete_bundle(tmp_path, https_server):
    manifest = write_manifest(tmp_path, https_server["url"])
    cache = tmp_path / "cache"
    retrievers = [make_retriever(cache, manifest) for _ in range(2)]
    barrier = threading.Barrier(2)

    def acquire(retriever):
        barrier.wait(timeout=5)
        return retriever.getFilePath(MODEL_URI)

    with ThreadPoolExecutor(max_workers=2) as executor:
        results = list(executor.map(acquire, retrievers))
    expected = bundle_path(cache, manifest)
    assert results == [str(expected / "robot.urdf")] * 2
    for path, body in FILES.items():
        assert (expected / path).read_bytes() == body
    assert list(expected.parent.iterdir()) == [expected]


def test_corrupt_published_cache_fails_until_explicitly_removed(tmp_path, https_server):
    manifest = write_manifest(tmp_path, https_server["url"])
    cache = tmp_path / "cache"
    online = make_retriever(cache, manifest)
    assert online.exists(MODEL_URI)
    bundle = bundle_path(cache, manifest)
    (bundle / "meshes/textures/body.png").write_bytes(b"corrupt")
    count = len(https_server["requests"])

    offline = make_retriever(cache, manifest, offline=True)
    assert not offline.exists(MODEL_URI)
    replacement = make_retriever(cache, manifest)
    assert not replacement.exists(MODEL_URI)
    assert len(https_server["requests"]) == count
    assert (bundle / "meshes/textures/body.png").read_bytes() == b"corrupt"


def test_python_package_resolution_uses_pinned_model_uri(tmp_path, https_server):
    manifest = write_manifest(tmp_path, https_server["url"])
    cache = tmp_path / "cache"
    retriever = make_retriever(cache, manifest)
    assert retriever.exists(MODEL_URI)
    packages = dart.utils.PackageResourceRetriever(retriever)
    packages.addPackageDirectory("testbot_description", "model://testbot/v1")
    path = packages.getFilePath("package://testbot_description/meshes/link.obj")
    assert Path(path) == bundle_path(cache, manifest) / "meshes/link.obj"
