function [] = load_spice_eros(NAIF_PATH)
if nargin<1
    NAIF_PATH = pwd;
end
cspice_furnsh(strcat(NAIF_PATH,'/naif/generic_kernels/lsk/naif0012.tls'));
cspice_furnsh(strcat(NAIF_PATH,'/naif/generic_kernels/pck/pck00010.tpc'));
cspice_furnsh(strcat(NAIF_PATH,'/naif/generic_kernels/spk/planets/de435.bsp'));

cspice_furnsh(strcat(NAIF_PATH,'/naif/generic_kernels/pck/gm_de431.tpc'));

%cspice_furnsh(strcat(NAIF_PATH,'/naif/generic_kernels/spk/asteroids/codes_300ast_20100725.bsp'));
cspice_furnsh(strcat(NAIF_PATH,'/naif/generic_kernels/spk/asteroids/a433.bsp'));