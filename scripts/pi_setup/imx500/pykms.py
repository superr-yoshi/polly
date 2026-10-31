"""
POLI: pykms 자리표시(stub).

Ubuntu에는 kmsxx(pykms)가 없다. Picamera2가 import할 때만 필요하고,
우리는 DrmPreview를 쓰지 않는다 (화면은 OpenCV 창). 실제로 쓰면 오류를 낸다.
install_picamera2.sh가 ~/.local/lib/python3.12/site-packages/에 복사한다.
"""


class _Stub:
    def __getattr__(self, name):
        return _Stub()

    def __call__(self, *args, **kwargs):
        raise RuntimeError('pykms stub: DRM preview is not available on this Ubuntu Pi')


def __getattr__(name):
    return _Stub()
