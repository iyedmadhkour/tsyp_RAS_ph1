from living_map_command_post.web import create_app
from living_map_command_post.state import CommandPostState

def test_gateway_app_exposes_state_and_mission_routes():
    app=create_app(CommandPostState(),lambda payload: True)
    paths={getattr(r,'path',None) for r in app.routes}
    assert {'/','/api/state','/api/mission','/ws'} <= paths


def test_source_dashboard_assets_are_resolvable():
    from living_map_command_post.web import _dashboard_dir
    web = _dashboard_dir()
    assert web is not None
    assert (web / "index.html").is_file()
    assert (web / "app.js").is_file()


def test_command_post_and_harness_views_are_separate(monkeypatch):
    import asyncio
    import anyio.to_thread
    import httpx

    def reject_worker_handoff(*args, **kwargs):
        raise AssertionError("ASGI command-post route used the broken worker-pool handoff")

    monkeypatch.setattr(anyio.to_thread, "run_sync", reject_worker_handoff)
    calls = []
    app = create_app(CommandPostState(), lambda payload: True, set_link=lambda up, after: calls.append((up, after)) or True)

    async def exercise():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
            assert (await client.get("/harness")).status_code == 200
            asset = await client.get("/static/app.js")
            assert asset.status_code == 200 and "LivingMap" in asset.text
            cut = await client.post("/api/harness/link", json={"up": False, "restore_after_s": 20})
            assert cut.status_code == 200 and "20 s" in cut.json()["message"]
            assert (await client.post("/api/harness/link", json={"up": True})).status_code == 200
            assert (await client.post("/api/harness/link", json={"up": False, "restore_after_s": -1})).status_code == 422
    asyncio.run(exercise())
    assert calls == [(False, 20.0), (True, None)]
