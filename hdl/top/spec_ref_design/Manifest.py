files = [
    "spec_ref_fmc_adc_100Ms.vhd",
    "spec_carrier_csr.vhd",
    "dma_eic.vhd",
]

fetchto = "../../ip_cores"

modules = {
    "local" : [
    "../../../",
    ],
    "git" : [
        "https://ohwr.org/project/general-cores.git",
        "https://ohwr.org/project/wr-cores.git",
        "https://ohwr.org/project/ddr3-sp6-core.git",
        "https://ohwr.org/project/gn4124-core.git",
        "https://ohwr.org/project/spec.git",
    ],
}
