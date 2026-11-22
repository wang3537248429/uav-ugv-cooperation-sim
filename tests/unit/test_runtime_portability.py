"""Catch missing model assets before Gazebo silently uses a local cache."""

import pytest

from tools.check_runtime import check_model_resources


@pytest.fixture
def simulation_share(tmp_path):
    (tmp_path / 'worlds').mkdir()
    (tmp_path / 'worlds/competition_world.sdf').write_text(
        '<sdf><world><include><uri>model://drone</uri></include></world></sdf>')
    model = tmp_path / 'models/drone'
    model.mkdir(parents=True)
    (model / 'model.sdf').write_text(
        '<sdf><model><uri>model://drone/body.dae</uri>'
        '<albedo_map>texture.png</albedo_map></model></sdf>')
    (model / 'body.dae').write_text('mesh')
    (model / 'texture.png').write_bytes(b'texture')
    return tmp_path


def test_complete_assets_need_no_external_model_cache(simulation_share):
    check_model_resources(simulation_share)


@pytest.mark.parametrize('missing', ['body.dae', 'texture.png'])
def test_missing_mesh_or_texture_fails_before_launch(simulation_share, missing):
    (simulation_share / 'models/drone' / missing).unlink()
    with pytest.raises(FileNotFoundError, match='Missing bundled'):
        check_model_resources(simulation_share)


def test_missing_included_model_fails_before_launch(simulation_share):
    (simulation_share / 'worlds/competition_world.sdf').write_text(
        '<sdf><world><include><uri>model://missing</uri></include></world></sdf>')
    with pytest.raises(FileNotFoundError, match='model://missing'):
        check_model_resources(simulation_share)
