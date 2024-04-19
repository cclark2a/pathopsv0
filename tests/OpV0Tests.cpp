#include "OpSkiaTests.h"

static void issue12556(skiatest::Reporter* reporter, const char* filename) {
SkPath patha;
      patha.moveTo(SkBits2Float(0x44f61000), SkBits2Float(0x45ce1700));  // 1968.5f, 6594.88f
      patha.lineTo(SkBits2Float(0x45c4d9ab), SkBits2Float(0x45ce1700));  // 6299.21f, 6594.88f
      patha.lineTo(SkBits2Float(0x45c49700), SkBits2Float(0x45ce59ab));  // 6290.88f, 6603.21f
      patha.lineTo(SkBits2Float(0x44f71aab), SkBits2Float(0x45ce59ab));  // 1976.83f, 6603.21f
      patha.lineTo(SkBits2Float(0x44f71aba), SkBits2Float(0x45e061e5));  // 1976.84f, 7180.24f
      patha.lineTo(SkBits2Float(0x45c4970f), SkBits2Float(0x45e061e5));  // 6290.88f, 7180.24f
      patha.lineTo(SkBits2Float(0x45c4d9be), SkBits2Float(0x45e0a493));  // 6299.22f, 7188.57f
      patha.lineTo(SkBits2Float(0x44f61000), SkBits2Float(0x45e0a493));  // 1968.5f, 7188.57f
      patha.lineTo(SkBits2Float(0x44f61000), SkBits2Float(0x45ce1700));  // 1968.5f, 6594.88f
      patha.close();
SkPath pathb;
      pathb.moveTo(SkBits2Float(0x45c4d9be), SkBits2Float(0x45ce1700));  // 6299.22f, 6594.88f
      pathb.lineTo(SkBits2Float(0x45c4d9be), SkBits2Float(0x45e0a493));  // 6299.22f, 7188.57f
      pathb.lineTo(SkBits2Float(0x45c4970f), SkBits2Float(0x45e061e5));  // 6290.88f, 7180.24f
      pathb.lineTo(SkBits2Float(0x45c4970f), SkBits2Float(0x45ce59af));  // 6290.88f, 6603.21f
      pathb.lineTo(SkBits2Float(0x45c4d9be), SkBits2Float(0x45ce1700));  // 6299.22f, 6594.88f
      pathb.close();
SkPathOp op = kUnion_SkPathOp;
    testPathOp(reporter, patha, pathb, op, filename);
}

static void pentrek1(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo({441.505f,264.541f});
b.quadTo({465.507f,252.904f}, {486.118f,246.702f});
b.quadTo({506.339f,240.617f}, {538.643f,233.581f});
b.quadTo({570.923f,226.55f}, {605.627f,221.6f});
b.quadTo({640.469f,216.631f}, {668.496f,215.479f});
b.quadTo({696.491f,214.329f}, {713.175f,215.219f});
b.quadTo({730.026f,216.119f}, {740.911f,218.108f});
b.cubicTo({740.911f,218.108f}, {746.341f,224.314f}, {749.943f,224.314f});
b.cubicTo({748.95f,229.743f}, {747.958f,238.775f}, {742.744f,238.775f});
b.quadTo({727.685f,236.022f}, {712.11f,235.191f});
b.quadTo({696.368f,234.351f}, {669.317f,235.462f});
b.quadTo({642.296f,236.572f}, {608.451f,241.4f});
b.quadTo({574.468f,246.247f}, {542.9f,253.122f});
b.quadTo({511.356f,259.993f}, {491.882f,265.853f});
b.quadTo({472.797f,271.596f}, {450.23f,282.537f});
b.cubicTo({450.23f,282.537f}, {445.263f,282.868f}, {439.277f,282.868f});
b.cubicTo({436.869f,277.902f}, {434.461f,266.949f}, {436.538f,266.949f});
b.close();
SkPath left(b);
b.reset();
b.moveTo({457.062f,207.276f});
b.quadTo({459.401f,218.797f}, {459.756f,227.626f});
b.quadTo({460.073f,235.527f}, {460.799f,246.604f});
b.quadTo({461.449f,256.519f}, {466.945f,272.989f});
b.quadTo({472.598f,289.93f}, {483.772f,311.371f});
b.quadTo({495.058f,333.025f}, {505.284f,348.272f});
b.quadTo({515.746f,363.871f}, {521.259f,371.657f});
b.quadTo({525.983f,378.328f}, {533.581f,384.209f});
b.cubicTo({533.581f,384.209f}, {537.946f,393.873f}, {538.747f,393.873f});
b.cubicTo({535.369f,398.238f}, {531.991f,403.403f}, {525.705f,403.403f});
b.quadTo({511.384f,392.32f}, {504.936f,383.214f});
b.quadTo({499.277f,375.222f}, {488.673f,359.412f});
b.quadTo({477.833f,343.248f}, {466.036f,320.614f});
b.quadTo({454.129f,297.766f}, {447.973f,279.32f});
b.quadTo({441.66f,260.402f}, {440.842f,247.912f});
b.quadTo({440.099f,236.582f}, {439.772f,228.428f});
b.quadTo({439.482f,221.203f}, {437.462f,211.256f});
b.cubicTo({437.462f,211.256f}, {436.363f,200.564f}, {439.863f,200.564f});
b.cubicTo({445.272f,199.466f}, {450.681f,201.867f}, {455.963f,201.867f});
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}


static void pentrek2(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo({0x1.b98132p+8,0x1.088a72p+8});
b.quadTo({0x1.d181dcp+8,0x1.f9cedp+7}, {0x1.e61e5p+8,0x1.ed672ep+7});
b.quadTo({0x1.fa56c8p+8,0x1.e13bb8p+7}, {0x1.0d525ap+9,0x1.d3294ap+7});
b.quadTo({0x1.1d7622p+9,0x1.c5199ep+7}, {0x1.2ed044p+9,0x1.bb334cp+7});
b.quadTo({0x1.403c1p+9,0x1.b142e6p+7}, {0x1.4e3f74p+9,0x1.aef55p+7});
b.quadTo({0x1.5c3edap+9,0x1.aca864p+7}, {0x1.649676p+9,0x1.ae7046p+7});
b.quadTo({0x1.6d034ap+9,0x1.b03cbp+7}, {0x1.7274a8p+9,0x1.b43772p+7});
b.cubicTo({0x1.7274a8p+9,0x1.b43772p+7}, {0x1.752b98p+9,0x1.c0a0dep+7}, {0x1.76f8aap+9,0x1.c0a0dep+7});
b.cubicTo({0x1.7679a4p+9,0x1.cb7c9ep+7}, {0x1.75fa9cp+9,0x1.dd8ca8p+7}, {0x1.735f48p+9,0x1.dd8ca8p+7});
b.quadTo({0x1.6bd7b6p+9,0x1.d80b5p+7}, {0x1.640e0ap+9,0x1.d661bap+7});
b.quadTo({0x1.5c2f26p+9,0x1.d4b39cp+7}, {0x1.4ea88cp+9,0x1.d6ecbp+7});
b.quadTo({0x1.4125fp+9,0x1.d9251ap+7}, {0x1.3039bcp+9,0x1.e2ccb4p+7});
b.quadTo({0x1.1f3bdep+9,0x1.ec7e62p+7}, {0x1.0f7326p+9,0x1.fa3eb6p+7});
b.quadTo({0x1.ff5b38p+8,0x1.03fe24p+8}, {0x1.ebe1bp+8,0x1.09da6ap+8});
b.quadTo({0x1.d8cc24p+8,0x1.0f9898p+8}, {0x1.c23acep+8,0x1.1a898ep+8});
b.cubicTo({0x1.c23acep+8,0x1.1a898ep+8}, {0x1.bd437p+8,0x1.1ade2cp+8}, {0x1.b746d4p+8,0x1.1ade2cp+8});
b.cubicTo({0x1.b4de72p+8,0x1.15e6cep+8}, {0x1.b27612p+8,0x1.0af2d4p+8}, {0x1.b489d4p+8,0x1.0af2d4p+8});
b.close();
SkPath left(b);
b.reset();
b.moveTo({0x1.c90fcep+8,0x1.9e8d24p+7});
b.quadTo({0x1.cb66b2p+8,0x1.b597fp+7}, {0x1.cbc17p+8,0x1.c7409ep+7});
b.quadTo({0x1.cc12a4p+8,0x1.d70ddp+7}, {0x1.cccc84p+8,0x1.ed351ap+7});
b.quadTo({0x1.cd72eap+8,0x1.0084f6p+8}, {0x1.d2f1dcp+8,0x1.10fd28p+8});
b.quadTo({0x1.d8991p+8,0x1.21edfap+8}, {0x1.e3c5bp+8,0x1.375edap+8});
b.quadTo({0x1.ef0ed8p+8,0x1.4d067ap+8}, {0x1.f94898p+8,0x1.5c458ep+8});
b.quadTo({0x1.01df82p+9,0x1.6bdf18p+8}, {0x1.04a12p+9,0x1.73a81cp+8});
b.quadTo({0x1.06fdd6p+9,0x1.7a5418p+8}, {0x1.0aca68p+9,0x1.80357ap+8});
b.cubicTo({0x1.0aca68p+9,0x1.80357ap+8}, {0x1.0cf918p+9,0x1.89df74p+8}, {0x1.0d5fa2p+9,0x1.89df74p+8});
b.cubicTo({0x1.0baf42p+9,0x1.8e3cd2p+8}, {0x1.09fee2p+9,0x1.936746p+8}, {0x1.06da46p+9,0x1.936746p+8});
b.quadTo({0x1.ff6252p+8,0x1.8851e8p+8}, {0x1.f8efbep+8,0x1.7f36e4p+8});
b.quadTo({0x1.f346fcp+8,0x1.7738e8p+8}, {0x1.e8ac68p+8,0x1.676972p+8});
b.quadTo({0x1.ddd528p+8,0x1.573f86p+8}, {0x1.d2095p+8,0x1.409d26p+8});
b.quadTo({0x1.c620fp+8,0x1.29c406p+8}, {0x1.bff924p+8,0x1.1751d8p+8});
b.quadTo({0x1.b9a916p+8,0x1.04670ap+8}, {0x1.b8d77cp+8,0x1.efd2e6p+7});
b.quadTo({0x1.b8195cp+8,0x1.d92a3p+7}, {0x1.b7c59p+8,0x1.c8db62p+7});
b.quadTo({0x1.b77b4ep+8,0x1.ba681p+7}, {0x1.b57632p+8,0x1.a682dcp+7});
b.cubicTo({0x1.b57632p+8,0x1.a682dcp+7}, {0x1.b45d0ap+8,0x1.9120b8p+7}, {0x1.b7dceep+8,0x1.9120b8p+7});
b.cubicTo({0x1.bd4592p+8,0x1.8eee66p+7}, {0x1.c2ae38p+8,0x1.93bbdap+7}, {0x1.c7f6a4p+8,0x1.93bbdap+7});
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek3(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo({0x1.2cb41ep+9,0x1.4bab78p+8});
b.quadTo({0x1.27abb6p+9,0x1.51fef6p+8}, {0x1.255c2p+9,0x1.55dce8p+8});
b.quadTo({0x1.230d52p+9,0x1.59b98ep+8}, {0x1.205328p+9,0x1.5fbc52p+8});
b.quadTo({0x1.1db57ep+9,0x1.658048p+8}, {0x1.1b7252p+9,0x1.6d2768p+8});
b.quadTo({0x1.195e7ap+9,0x1.742e7p+8}, {0x1.186614p+9,0x1.7e5ab6p+8});
b.quadTo({0x1.176b5p+9,0x1.889fbap+8}, {0x1.186e1cp+9,0x1.973346p+8});
b.quadTo({0x1.1974ccp+9,0x1.a5fee6p+8}, {0x1.1d4782p+9,0x1.b712e8p+8});
b.quadTo({0x1.2127b2p+9,0x1.c86326p+8}, {0x1.27101ep+9,0x1.d7d258p+8});
b.quadTo({0x1.2cfc84p+9,0x1.e74bf2p+8}, {0x1.330cc8p+9,0x1.f168fp+8});
b.quadTo({0x1.3911eep+9,0x1.fb735cp+8}, {0x1.3e2b4p+9,0x1.0030eap+9});
b.quadTo({0x1.43318p+9,0x1.029eeep+9}, {0x1.477c1cp+9,0x1.035f28p+9});
b.quadTo({0x1.4bbbdap+9,0x1.041d7ap+9}, {0x1.50b6e6p+9,0x1.038298p+9});
b.quadTo({0x1.55e63p+9,0x1.02e15cp+9}, {0x1.5b75dp+9,0x1.00f342p+9});
b.quadTo({0x1.60f6bep+9,0x1.fe1486p+8}, {0x1.667374p+9,0x1.f706ecp+8});
b.quadTo({0x1.6be7f6p+9,0x1.f003ep+8}, {0x1.70186ap+9,0x1.e6f964p+8});
b.quadTo({0x1.74029ep+9,0x1.de8688p+8}, {0x1.75534p+9,0x1.d70c2cp+8});
b.quadTo({0x1.76b608p+9,0x1.cf2a8ep+8}, {0x1.76c3d6p+9,0x1.c5e04ap+8});
b.quadTo({0x1.76d2aep+9,0x1.bbe25ap+8}, {0x1.75dd7ep+9,0x1.b12c28p+8});
b.quadTo({0x1.74f346p+9,0x1.a6f0b8p+8}, {0x1.7302b4p+9,0x1.9f2422p+8});
b.quadTo({0x1.71176ap+9,0x1.976ccap+8}, {0x1.6e0e4cp+9,0x1.9117aep+8});
b.quadTo({0x1.6ad6c6p+9,0x1.8a61c6p+8}, {0x1.669ed8p+9,0x1.84867ep+8});
b.quadTo({0x1.622e86p+9,0x1.7e5ceap+8}, {0x1.5e7a24p+9,0x1.7a2e76p+8});
b.quadTo({0x1.5ab122p+9,0x1.75e8b8p+8}, {0x1.5737cep+9,0x1.727c6cp+8});
b.cubicTo({0x1.5737cep+9,0x1.727c6cp+8}, {0x1.54be18p+9,0x1.6a0c0ap+8}, {0x1.53b912p+9,0x1.6a0c0ap+8});
b.cubicTo({0x1.54f14ap+9,0x1.65189cp+8}, {0x1.562982p+9,0x1.5e1b26p+8}, {0x1.59297ap+9,0x1.5e1b26p+8});
b.quadTo({0x1.5f5cdep+9,0x1.643748p+8}, {0x1.63645cp+9,0x1.68c38ap+8});
b.quadTo({0x1.67807ap+9,0x1.6d6716p+8}, {0x1.6c52a8p+9,0x1.741882p+8});
b.quadTo({0x1.715d3ap+9,0x1.7b183ap+8}, {0x1.754634p+9,0x1.834052p+8});
b.quadTo({0x1.795d96p+9,0x1.8bc936p+8}, {0x1.7bf6ccp+9,0x1.963bdep+8});
b.quadTo({0x1.7e8abap+9,0x1.a09948p+8}, {0x1.7fb582p+9,0x1.ada6d8p+8});
b.quadTo({0x1.80d552p+9,0x1.ba39a6p+8}, {0x1.80c3aap+9,0x1.c61bb6p+8});
b.quadTo({0x1.80b0f8p+9,0x1.d2b172p+8}, {0x1.7ec24p+9,0x1.ddaed4p+8});
b.quadTo({0x1.7cc162p+9,0x1.e91378p+8}, {0x1.776e16p+9,0x1.f4919cp+8});
b.quadTo({0x1.72610ap+9,0x1.ff782p+8}, {0x1.6bdb8cp+9,0x1.03ed0ap+9});
b.quadTo({0x1.655e42p+9,0x1.0818bcp+9}, {0x1.5ebd3p+9,0x1.0a65bep+9});
b.quadTo({0x1.582adp+9,0x1.0cada4p+9}, {0x1.51eb9ap+9,0x1.0d6fe8p+9});
b.quadTo({0x1.4b7826p+9,0x1.0e3886p+9}, {0x1.45c2e4p+9,0x1.0d38d8p+9});
b.quadTo({0x1.40188p+9,0x1.0c3b12p+9}, {0x1.39d0cp+9,0x1.093196p+9});
b.quadTo({0x1.339c12p+9,0x1.063152p+9}, {0x1.2ca538p+9,0x1.006288p+9});
b.quadTo({0x1.25b97cp+9,0x1.f53a0ep+8}, {0x1.1f1f62p+9,0x1.e3faa8p+8});
b.quadTo({0x1.18814ep+9,0x1.d2b0dap+8}, {0x1.1426fep+9,0x1.bf3f18p+8});
b.quadTo({0x1.0fbf34p+9,0x1.ab911ap+8}, {0x1.0e8664p+9,0x1.99f2bap+8});
b.quadTo({0x1.0d49bp+9,0x1.881c46p+8}, {0x1.0e936cp+9,0x1.7a9b4ap+8});
b.quadTo({0x1.0fdf86p+9,0x1.6d019p+8}, {0x1.12d6aep+9,0x1.62f998p+8});
b.quadTo({0x1.159e82p+9,0x1.5991b8p+8}, {0x1.18eb58p+9,0x1.524baep+8});
b.quadTo({0x1.1c1baep+9,0x1.4b4472p+8}, {0x1.1ef16p+9,0x1.468618p+8});
b.quadTo({0x1.21c64ap+9,0x1.41c90ap+8}, {0x1.2761e2p+9,0x1.3abc88p+8});
b.cubicTo({0x1.2761e2p+9,0x1.3abc88p+8}, {0x1.29b804p+9,0x1.393582p+8}, {0x1.2cced2p+9,0x1.393582p+8});
b.cubicTo({0x1.2e46bcp+9,0x1.3de1c6p+8}, {0x1.2fbea8p+9,0x1.48bba4p+8}, {0x1.2f0a3ep+9,0x1.48bba4p+8});
b.close();
SkPath left(b);
b.reset();
b.moveTo({0x1.2cee94p+9,0x1.b0d972p+8});
b.quadTo({0x1.30d7acp+9,0x1.a625ecp+8}, {0x1.325f6ap+9,0x1.a1b576p+8});
b.quadTo({0x1.33f1ap+9,0x1.9d269ep+8}, {0x1.35dfd8p+9,0x1.9790c8p+8});
b.quadTo({0x1.37ee56p+9,0x1.919d88p+8}, {0x1.3a26b6p+9,0x1.8c498ep+8});
b.quadTo({0x1.3c8a1p+9,0x1.868e68p+8}, {0x1.3ebf2cp+9,0x1.82f936p+8});
b.quadTo({0x1.4151eep+9,0x1.7ecbfep+8}, {0x1.45031cp+9,0x1.7c7d18p+8});
b.quadTo({0x1.4904ccp+9,0x1.79fbdcp+8}, {0x1.4cf898p+9,0x1.7b98a8p+8});
b.quadTo({0x1.50c5c2p+9,0x1.7d25b2p+8}, {0x1.536cd4p+9,0x1.812844p+8});
b.quadTo({0x1.559c82p+9,0x1.84765ap+8}, {0x1.5803dcp+9,0x1.8a1d9cp+8});
b.quadTo({0x1.5a33b2p+9,0x1.8f425p+8}, {0x1.5cda52p+9,0x1.96b3cep+8});
b.quadTo({0x1.5f560ap+9,0x1.9dacd8p+8}, {0x1.62dedp+9,0x1.a7000ep+8});
b.quadTo({0x1.663d7ep+9,0x1.afe434p+8}, {0x1.6ba1eep+9,0x1.baa47cp+8});
b.quadTo({0x1.710152p+9,0x1.c55aaep+8}, {0x1.77c024p+9,0x1.cf4818p+8});
b.quadTo({0x1.7e9138p+9,0x1.d95062p+8}, {0x1.8619fep+9,0x1.e1f228p+8});
b.quadTo({0x1.8dbc02p+9,0x1.eab0d8p+8}, {0x1.95a85ap+9,0x1.f25f68p+8});
b.quadTo({0x1.9d8328p+9,0x1.f9fcf2p+8}, {0x1.a4a97cp+9,0x1.ff3e18p+8});
b.quadTo({0x1.abcfa2p+9,0x1.023f8ep+9}, {0x1.b2ace6p+9,0x1.04051ep+9});
b.quadTo({0x1.b99a6p+9,0x1.05cedep+9}, {0x1.bf3c1ep+9,0x1.06d3fep+9});
b.quadTo({0x1.c4e942p+9,0x1.07db2ep+9}, {0x1.c8721ap+9,0x1.084bcep+9});
b.quadTo({0x1.cbfd72p+9,0x1.08bcbcp+9}, {0x1.cdf622p+9,0x1.08e15cp+9});
b.quadTo({0x1.cee07ap+9,0x1.08f25cp+9}, {0x1.cfc68ep+9,0x1.0892cp+9});
b.quadTo({0x1.cf8524p+9,0x1.08adeep+9}, {0x1.cf7992p+9,0x1.08d7c8p+9});
b.quadTo({0x1.cfe046p+9,0x1.076474p+9}, {0x1.cfbf32p+9,0x1.0376d6p+9});
b.quadTo({0x1.cf9b54p+9,0x1.fe694p+8}, {0x1.ce41ep+9,0x1.f172dp+8});
b.quadTo({0x1.ccdc26p+9,0x1.e4068p+8}, {0x1.cadf04p+9,0x1.d665c2p+8});
b.quadTo({0x1.c8d304p+9,0x1.c85f22p+8}, {0x1.c75308p+9,0x1.bdee8p+8});
b.quadTo({0x1.c5c9aap+9,0x1.b33c92p+8}, {0x1.c50992p+9,0x1.ac9b18p+8});
b.quadTo({0x1.c4562ap+9,0x1.a669ap+8}, {0x1.c3c7e8p+9,0x1.a22658p+8});
b.quadTo({0x1.c363dap+9,0x1.9f26d4p+8}, {0x1.c27726p+9,0x1.9c1d2p+8});
b.quadTo({0x1.c1bb02p+9,0x1.99b302p+8}, {0x1.bf19fep+9,0x1.962bb6p+8});
b.quadTo({0x1.bc08eap+9,0x1.920e06p+8}, {0x1.b5e066p+9,0x1.8d601cp+8});
b.quadTo({0x1.af9436p+9,0x1.889716p+8}, {0x1.a8944cp+9,0x1.85d666p+8});
b.quadTo({0x1.a1d42cp+9,0x1.832ecep+8}, {0x1.9dd89p+9,0x1.83c848p+8});
b.quadTo({0x1.9adb04p+9,0x1.843b84p+8}, {0x1.99d0f6p+9,0x1.860ee6p+8});
b.quadTo({0x1.9815a8p+9,0x1.8919a6p+8}, {0x1.964476p+9,0x1.8f00b8p+8});
b.quadTo({0x1.941d6cp+9,0x1.95fe9cp+8}, {0x1.918a0ap+9,0x1.9f4338p+8});
b.quadTo({0x1.8eeab4p+9,0x1.a8b2dap+8}, {0x1.8c6b16p+9,0x1.b1d1f4p+8});
b.quadTo({0x1.89ea4cp+9,0x1.baf55ap+8}, {0x1.881e82p+9,0x1.c18d3ep+8});
b.quadTo({0x1.8659dp+9,0x1.c80b1ep+8}, {0x1.854bbep+9,0x1.cc299ap+8});
b.quadTo({0x1.844a78p+9,0x1.d01622p+8}, {0x1.834aa8p+9,0x1.d4c28cp+8});
b.quadTo({0x1.81eefcp+9,0x1.db1c9ep+8}, {0x1.7fa662p+9,0x1.e08f38p+8});
b.quadTo({0x1.7cd7bcp+9,0x1.e74192p+8}, {0x1.74a244p+9,0x1.ebce0ep+8});
b.cubicTo({0x1.74a244p+9,0x1.ebce0ep+8}, {0x1.71f978p+9,0x1.ea2826p+8}, {0x1.6f379ap+9,0x1.ea2826p+8});
b.cubicTo({0x1.6e7af8p+9,0x1.e4d68ap+8}, {0x1.6dbe58p+9,0x1.da0132p+8}, {0x1.6f4deep+9,0x1.da0132p+8});
b.quadTo({0x1.772d44p+9,0x1.d5a46ep+8}, {0x1.77fc9ep+9,0x1.d3b5c8p+8});
b.quadTo({0x1.795204p+9,0x1.d08762p+8}, {0x1.7a18d8p+9,0x1.cce574p+8});
b.quadTo({0x1.7b3b88p+9,0x1.c795dep+8}, {0x1.7c6542p+9,0x1.c30b66p+8});
b.quadTo({0x1.7d823p+9,0x1.beb2e2p+8}, {0x1.7f567ep+9,0x1.b7fbc2p+8});
b.quadTo({0x1.8123b4p+9,0x1.b15ea6p+8}, {0x1.83a5eap+9,0x1.a8360cp+8});
b.quadTo({0x1.86294cp+9,0x1.9f0926p+8}, {0x1.88cc76p+9,0x1.958bc8p+8});
b.quadTo({0x1.8b7b94p+9,0x1.8be364p+8}, {0x1.8dc08ap+9,0x1.848448p+8});
b.quadTo({0x1.905b58p+9,0x1.7c0e5ap+8}, {0x1.93378ap+9,0x1.77081ap+8});
b.quadTo({0x1.96c4fcp+9,0x1.70ca7cp+8}, {0x1.9d187p+9,0x1.6fd6b8p+8});
b.quadTo({0x1.a26dd4p+9,0x1.6f0932p+8}, {0x1.aa8234p+9,0x1.72369ap+8});
b.quadTo({0x1.b256cap+9,0x1.754aeap+8}, {0x1.b96d9ap+9,0x1.7aade4p+8});
b.quadTo({0x1.c0a816p+9,0x1.802bfap+8}, {0x1.c4ac82p+9,0x1.85904ap+8});
b.quadTo({0x1.c920fep+9,0x1.8b8afep+8}, {0x1.cb01dap+9,0x1.91b6ep+8});
b.quadTo({0x1.ccb226p+9,0x1.97432cp+8}, {0x1.cd7518p+9,0x1.9d1aa8p+8});
b.quadTo({0x1.ce0dd6p+9,0x1.a1ae6p+8}, {0x1.ceca6ep+9,0x1.a830e8p+8});
b.quadTo({0x1.cf7a56p+9,0x1.ae436ep+8}, {0x1.d0ef78p+9,0x1.b8688p+8});
b.quadTo({0x1.d26dfcp+9,0x1.c2cedep+8}, {0x1.d4787cp+9,0x1.d0cb3ep+8});
b.quadTo({0x1.d691dap+9,0x1.df2d8p+8}, {0x1.d80c2p+9,0x1.ed5f3p+8});
b.quadTo({0x1.d992acp+9,0x1.fc06cp+8}, {0x1.d9bdcep+9,0x1.0322aap+9});
b.quadTo({0x1.d9ebbap+9,0x1.08968cp+9}, {0x1.d91ceep+9,0x1.0b8238p+9});
b.quadTo({0x1.d7dbdcp+9,0x1.100b12p+9}, {0x1.d39cf2p+9,0x1.11cecp+9});
b.quadTo({0x1.d08586p+9,0x1.1317a4p+9}, {0x1.cd3cdep+9,0x1.12daa4p+9});
b.quadTo({0x1.cb028ep+9,0x1.12b144p+9}, {0x1.c735e6p+9,0x1.123832p+9});
b.quadTo({0x1.c366bep+9,0x1.11bed2p+9}, {0x1.bd73e2p+9,0x1.10ab02p+9});
b.quadTo({0x1.b775ap+9,0x1.0f9522p+9}, {0x1.b02d1ap+9,0x1.0db3e2p+9});
b.quadTo({0x1.a8d45ep+9,0x1.0bce72p+9}, {0x1.a13684p+9,0x1.0901f4p+9});
b.quadTo({0x1.9998d8p+9,0x1.063588p+9}, {0x1.914ba6p+9,0x1.022f4cp+9});
b.quadTo({0x1.890ffep+9,0x1.fc6328p+8}, {0x1.812182p+9,0x1.f34cd8p+8});
b.quadTo({0x1.7919c8p+9,0x1.ea199ep+8}, {0x1.71d2dcp+9,0x1.df63e8p+8});
b.quadTo({0x1.6a79aep+9,0x1.d49352p+8}, {0x1.649292p+9,0x1.c8ce84p+8});
b.quadTo({0x1.5eb082p+9,0x1.bd13ccp+8}, {0x1.5ae6bp+9,0x1.b314f2p+8});
b.quadTo({0x1.5746f6p+9,0x1.a98528p+8}, {0x1.54b52ep+9,0x1.a24e32p+8});
b.quadTo({0x1.524e4ep+9,0x1.9b8fbp+8}, {0x1.5065a4p+9,0x1.971264p+8});
b.quadTo({0x1.4eb47ep+9,0x1.9317a6p+8}, {0x1.4d652cp+9,0x1.911cbcp+8});
b.quadTo({0x1.4c8d3ep+9,0x1.8fd64ep+8}, {0x1.4af8e8p+9,0x1.8f3158p+8});
b.quadTo({0x1.498b34p+9,0x1.8e9c24p+8}, {0x1.47fee4p+9,0x1.8f93e8p+8});
b.quadTo({0x1.462212p+9,0x1.90be02p+8}, {0x1.450c54p+9,0x1.9280cap+8});
b.quadTo({0x1.4398fp+9,0x1.94db98p+8}, {0x1.41d54ap+9,0x1.991772p+8});
b.quadTo({0x1.3fe6aap+9,0x1.9dba78p+8}, {0x1.3e19a8p+9,0x1.a2f038p+8});
b.quadTo({0x1.3c2c6p+9,0x1.a88362p+8}, {0x1.3a9b16p+9,0x1.ad0f8ap+8});
b.quadTo({0x1.38ff54p+9,0x1.b1ba14p+8}, {0x1.35016cp+9,0x1.bca68ep+8});
b.cubicTo({0x1.35016cp+9,0x1.bca68ep+8}, {0x1.336092p+9,0x1.c2148ap+8}, {0x1.303f16p+9,0x1.c2148ap+8});
b.cubicTo({0x1.2e04b8p+9,0x1.bed2d6p+8}, {0x1.2bca5cp+9,0x1.b54e2ap+8}, {0x1.2b4dbcp+9,0x1.b54e2ap+8});
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek4(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.d8dfb2p+8, 0x1.8cc12ap+7);
b.quadTo(0x1.d393f8p+8, 0x1.9e5cbep+7, 0x1.d1d146p+8, 0x1.a5901ep+7);
b.quadTo(0x1.d00c96p+8, 0x1.accbaap+7, 0x1.cde8c6p+8, 0x1.b7c716p+7);
b.quadTo(0x1.cbdc68p+8, 0x1.c24a3ep+7, 0x1.ca35c4p+8, 0x1.cfe4bp+7);
b.quadTo(0x1.c89fccp+8, 0x1.dcf5c8p+7, 0x1.c833d4p+8, 0x1.ede2d6p+7);
b.quadTo(0x1.c7c7eap+8, 0x1.fecdep+7, 0x1.c916b2p+8, 0x1.08ba84p+8);
b.quadTo(0x1.ca52fap+8, 0x1.118a18p+8, 0x1.cebbdcp+8, 0x1.1b20eep+8);
b.quadTo(0x1.d330b8p+8, 0x1.24d1dp+8, 0x1.daf9bap+8, 0x1.2e0c4cp+8);
b.quadTo(0x1.e2942ap+8, 0x1.370f92p+8, 0x1.ec7eep+8, 0x1.3d1302p+8);
b.quadTo(0x1.f69daap+8, 0x1.433606p+8, 0x1.018452p+9, 0x1.469a7ep+8);
b.quadTo(0x1.07c722p+9, 0x1.4a063ep+8, 0x1.0f999ap+9, 0x1.4a2c1ep+8);
b.quadTo(0x1.179db6p+9, 0x1.4a52ecp+8, 0x1.200a82p+9, 0x1.476966p+8);
b.quadTo(0x1.286ed6p+9, 0x1.4482cep+8, 0x1.2dcf4cp+9, 0x1.4066bp+8);
b.quadTo(0x1.32a8d2p+9, 0x1.3cb1b6p+8, 0x1.34263cp+9, 0x1.39c036p+8);
b.quadTo(0x1.35044ap+9, 0x1.380988p+8, 0x1.351308p+9, 0x1.35fed6p+8);
b.quadTo(0x1.352dc6p+9, 0x1.324a92p+8, 0x1.340786p+9, 0x1.2c6f7cp+8);
b.quadTo(0x1.32d97ep+9, 0x1.266cc4p+8, 0x1.2e5e74p+9, 0x1.1d3b7ep+8);
b.quadTo(0x1.2990d4p+9, 0x1.1360ccp+8, 0x1.21c1dep+9, 0x1.0925f6p+8);
b.quadTo(0x1.1a03bcp+9, 0x1.fe0254p+7, 0x1.10c0ep+9, 0x1.f14a92p+7);
b.quadTo(0x1.074c28p+9, 0x1.e44e58p+7, 0x1.feb2bp+8, 0x1.df142cp+7);
b.quadTo(0x1.ef22c4p+8, 0x1.d9f63p+7, 0x1.e8749p+8, 0x1.daeccp+7);
b.quadTo(0x1.e31764p+8, 0x1.dbb2b8p+7, 0x1.e1b5f8p+8, 0x1.dd6fcp+7);
b.quadTo(0x1.dfc076p+8, 0x1.dfe742p+7, 0x1.dc5932p+8, 0x1.e8ddaap+7);
b.quadTo(0x1.d84a4ap+8, 0x1.f38d86p+7, 0x1.d256d6p+8, 0x1.049736p+8);
b.cubicTo(0x1.d256d6p+8, 0x1.049736p+8, 0x1.cfada8p+8, 0x1.0b3104p+8, 0x1.c997a6p+8, 0x1.0b3104p+8);
b.cubicTo(0x1.c4c1cap+8, 0x1.0887d6p+8, 0x1.bfebeep+8, 0x1.ff914ap+7, 0x1.be27fcp+8, 0x1.ff914ap+7);
b.quadTo(0x1.c779b6p+8, 0x1.ddb27ap+7, 0x1.cc6bcep+8, 0x1.d0ac56p+7);
b.quadTo(0x1.d2058ap+8, 0x1.c1ecbep+7, 0x1.d70e08p+8, 0x1.bb964p+7);
b.quadTo(0x1.dcaa9cp+8, 0x1.b48548p+7, 0x1.e7047p+8, 0x1.b3074p+7);
b.quadTo(0x1.f00d3cp+8, 0x1.b1b9dp+7, 0x1.00f8a8p+9, 0x1.b79bd4p+7);
b.quadTo(0x1.09bfd8p+9, 0x1.bd61a8p+7, 0x1.14002p+9, 0x1.cb756ep+7);
b.quadTo(0x1.1e7244p+9, 0x1.d9cdacp+7, 0x1.273ca2p+9, 0x1.f0d614p+7);
b.quadTo(0x1.2ff62cp+9, 0x1.03d934p+8, 0x1.35878cp+9, 0x1.0f4582p+8);
b.quadTo(0x1.3b6b82p+9, 0x1.1b5b3cp+8, 0x1.3d567ap+9, 0x1.252084p+8);
b.quadTo(0x1.3f493ap+9, 0x1.2f0d6ep+8, 0x1.3f0ef8p+9, 0x1.371f2ap+8);
b.quadTo(0x1.3ec8b6p+9, 0x1.40da78p+8, 0x1.3b2d44p+9, 0x1.47facap+8);
b.quadTo(0x1.38312ep+9, 0x1.4de04ap+8, 0x1.316134p+9, 0x1.53155p+8);
b.quadTo(0x1.2b182ap+9, 0x1.57e332p+8, 0x1.21be7ep+9, 0x1.5b1e9ap+8);
b.quadTo(0x1.186d4ap+9, 0x1.5e5714p+8, 0x1.0f8166p+9, 0x1.5e2be2p+8);
b.quadTo(0x1.0663dep+9, 0x1.5dffc2p+8, 0x1.fdc35ap+8, 0x1.59e582p+8);
b.quadTo(0x1.eea456p+8, 0x1.55c3fap+8, 0x1.e2202p+8, 0x1.4e2cfep+8);
b.quadTo(0x1.d567d6p+8, 0x1.46766ep+8, 0x1.cbb046p+8, 0x1.3af1b4p+8);
b.quadTo(0x1.c22748p+8, 0x1.2fa43p+8, 0x1.bc9024p+8, 0x1.237c12p+8);
b.quadTo(0x1.b6ed06p+8, 0x1.1739e8p+8, 0x1.b5484ep+8, 0x1.0b817cp+8);
b.quadTo(0x1.b3b616p+8, 0x1.004d1p+8, 0x1.b43a2cp+8, 0x1.ebe52ap+7);
b.quadTo(0x1.b4be34p+8, 0x1.d73238p+7, 0x1.b6c63cp+8, 0x1.c6755p+7);
b.quadTo(0x1.b8bd98p+8, 0x1.b641c2p+7, 0x1.bb463ap+8, 0x1.a940eap+7);
b.quadTo(0x1.bdb76ap+8, 0x1.9cb856p+7, 0x1.bfd9bap+8, 0x1.93fde2p+7);
b.quadTo(0x1.c1fe08p+8, 0x1.8b3b42p+7, 0x1.c7bc4ep+8, 0x1.7822d6p+7);
b.cubicTo(0x1.c7bc4ep+8, 0x1.7822d6p+7, 0x1.ca949ap+8, 0x1.6b9e04p+7, 0x1.d0bad8p+8, 0x1.6b9e04p+7);
b.cubicTo(0x1.d57594p+8, 0x1.714e9cp+7, 0x1.da305p+8, 0x1.834bb2p+7, 0x1.dbb7fep+8, 0x1.834bb2p+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.9c6034p+8, 0x1.1e47bp+8);
b.quadTo(0x1.a8507ap+8, 0x1.27ffbcp+8, 0x1.b0e098p+8, 0x1.2dd55ap+8);
b.cubicTo(0x1.b0e098p+8, 0x1.2dd55ap+8, -0x1.670b34p+10, 0x1.8a539cp+10, 0x1.1f92e8p+11, 0x1.8a539cp+10);
b.quadTo(0x1.b9601ap+8, 0x1.33a304p+8, 0x1.c5bed8p+8, 0x1.3a7c7ap+8);
b.cubicTo(0x1.c5bed8p+8, 0x1.3a7c7ap+8, 0x1.d06b0ap+10, -0x1.d0606ep+8, -0x1.daabd4p+9, -0x1.d0606ep+8);
b.quadTo(0x1.d207bp+8, 0x1.414e32p+8, 0x1.e587f4p+8, 0x1.49a596p+8);
b.cubicTo(0x1.e587f4p+8, 0x1.49a596p+8, -0x1.957bccp+8, 0x1.63dd74p+9, 0x1.58095p+10, 0x1.63dd74p+9);
b.quadTo(0x1.f90bb6p+8, 0x1.5204d2p+8, 0x1.08ea28p+9, 0x1.5a4778p+8);
b.cubicTo(0x1.08ea28p+9, 0x1.5a4778p+8, -0x1.294f56p+12, 0x1.0843bap+11, 0x1.6b6b56p+12, 0x1.0843bap+11);
b.quadTo(0x1.152cfcp+9, 0x1.627b66p+8, 0x1.22734cp+9, 0x1.67f912p+8);
b.cubicTo(0x1.22734cp+9, 0x1.67f912p+8, -0x1.bf0342p+9, 0x1.4d5148p+9, 0x1.00f014p+11, 0x1.4d5148p+9);
b.quadTo(0x1.2f97d2p+9, 0x1.6d7076p+8, 0x1.3dfba2p+9, 0x1.6ef374p+8);
b.cubicTo(0x1.3dfba2p+9, 0x1.6ef374p+8, 0x1.28ce1p+11, 0x1.11b81p+8, -0x1.139944p+10, 0x1.11b81p+8);
b.quadTo(0x1.4c4356p+9, 0x1.707b94p+8, 0x1.57d58cp+9, 0x1.6da4cep+8);
b.cubicTo(0x1.57d58cp+9, 0x1.6da4cep+8, 0x1.438e82p+8, 0x1.4160fcp+8, 0x1.06f5c6p+10, 0x1.4160fcp+8);
b.quadTo(0x1.6320c2p+9, 0x1.6ae5f4p+8, 0x1.6b49bcp+9, 0x1.642a54p+8);
b.cubicTo(0x1.6b49bcp+9, 0x1.642a54p+8, -0x1.2c52ep+11, -0x1.d1acep+9, 0x1.e2256p+11, -0x1.d1acep+9);
b.quadTo(0x1.73639ep+9, 0x1.5d807ap+8, 0x1.791942p+9, 0x1.545e7ep+8);
b.cubicTo(0x1.791942p+9, 0x1.545e7ep+8, -0x1.73b9d4p+11, -0x1.49a80ep+11, 0x1.18560ap+12, -0x1.49a80ep+11);
b.quadTo(0x1.7e6c26p+9, 0x1.4bdf38p+8, 0x1.80a8f6p+9, 0x1.421f3cp+8);
b.quadTo(0x1.82e4dcp+9, 0x1.3869ap+8, 0x1.82a8cep+9, 0x1.2b12c4p+8);
b.quadTo(0x1.826e02p+9, 0x1.1da4c4p+8, 0x1.7dbb8cp+9, 0x1.0b5108p+8);
b.cubicTo(0x1.7dbb8cp+9, 0x1.0b5108p+8, 0x1.2aba3ap+10, -0x1.1f69c4p+9, 0x1.4cdddep+8, -0x1.1f69c4p+9);
b.quadTo(0x1.78f64ep+9, 0x1.f15052p+7, 0x1.6defdcp+9, 0x1.c8c4e2p+7);
b.cubicTo(0x1.6defdcp+9, 0x1.c8c4e2p+7, 0x1.231d64p+10, -0x1.539222p+7, 0x1.2bb3cep+8, -0x1.539222p+7);
b.quadTo(0x1.62c68cp+9, 0x1.9fa2d6p+7, 0x1.5651a6p+9, 0x1.851544p+7);
b.cubicTo(0x1.5651a6p+9, 0x1.851544p+7, -0x1.49f63cp+9, 0x1.c8406ep+9, 0x1.fb1b04p+10, 0x1.c8406ep+9);
b.quadTo(0x1.49ed84p+9, 0x1.6a99fep+7, 0x1.417218p+9, 0x1.62a7aap+7);
b.cubicTo(0x1.417218p+9, 0x1.62a7aap+7, -0x1.42ccdp+8, 0x1.94848ap+8, 0x1.921572p+10, 0x1.94848ap+8);
b.quadTo(0x1.39405ep+9, 0x1.5af11ap+7, 0x1.3507dap+9, 0x1.5d2f38p+7);
b.cubicTo(0x1.3507dap+9, 0x1.5d2f38p+7, 0x1.5a940ap+10, 0x1.13df3ap+8, -0x1.2c9aep+7, 0x1.13df3ap+8);
b.quadTo(0x1.315d12p+9, 0x1.5f1e3p+7, 0x1.2fe356p+9, 0x1.64173cp+7);
b.cubicTo(0x1.2fe356p+9, 0x1.64173cp+7, -0x1.45b8c2p+9, -0x1.b8c97cp+9, 0x1.d30768p+10, -0x1.b8c97cp+9);
b.quadTo(0x1.2e072p+9, 0x1.6a58bcp+7, 0x1.2c84fp+9, 0x1.758a2ap+7);
b.cubicTo(0x1.2c84fp+9, 0x1.758a2ap+7, -0x1.ab989p+7, -0x1.4aef8ep+10, 0x1.626a2ep+10, -0x1.4aef8ep+10);
b.quadTo(0x1.2abea8p+9, 0x1.82ab5ap+7, 0x1.29244ap+9, 0x1.94dd0ep+7);
b.cubicTo(0x1.29244ap+9, 0x1.94dd0ep+7, 0x1.f16ff8p+8, -0x1.20f58p+6, 0x1.59bb4p+9, -0x1.20f58p+6);
b.quadTo(0x1.277dbep+9, 0x1.a7863ap+7, 0x1.2653d8p+9, 0x1.baa55p+7);
b.cubicTo(0x1.2653d8p+9, 0x1.baa55p+7, 0x1.4d11f6p+9, 0x1.0dc622p+9, 0x1.fec3b8p+8, 0x1.0dc622p+9);
b.quadTo(0x1.2525fcp+9, 0x1.cdea6ep+7, 0x1.24649ep+9, 0x1.e0a46cp+7);
b.cubicTo(0x1.24649ep+9, 0x1.e0a46cp+7, 0x1.fa8836p+8, -0x1.e930bap+7, 0x1.4bd5d4p+9, -0x1.e930bap+7);
b.quadTo(0x1.23a3a6p+9, 0x1.f32d5ep+7, 0x1.235ce6p+9, 0x1.01cebp+8);
b.cubicTo(0x1.235ce6p+9, 0x1.01cebp+8, 0x1.558dd4p+9, 0x1.b5903cp+10, 0x1.e05ad6p+8, 0x1.b5903cp+10);
b.quadTo(0x1.2314e4p+9, 0x1.0a0222p+8, 0x1.23264cp+9, 0x1.0f7132p+8);
b.cubicTo(0x1.23264cp+9, 0x1.0f7132p+8, 0x1.2cb034p+9, -0x1.eb1e2cp+8, 0x1.1a2902p+9, -0x1.eb1e2cp+8);
b.quadTo(0x1.233716p+9, 0x1.15016p+8, 0x1.236002p+9, 0x1.18903ap+8);
b.cubicTo(0x1.236002p+9, 0x1.18903ap+8, 0x1.157016p+9, 0x1.2767bp+9, 0x1.311022p+9, 0x1.2767bp+9);
b.quadTo(0x1.23895p+9, 0x1.1c3856p+8, 0x1.240be6p+9, 0x1.23c09ap+8);
b.cubicTo(0x1.240be6p+9, 0x1.23c09ap+8, 0x1.246ab6p+9, 0x1.2e4364p+8, 0x1.227f54p+9, 0x1.2e4364p+8);
b.cubicTo(0x1.1fc346p+9, 0x1.2f0106p+8, 0x1.1d0736p+9, 0x1.2be7e6p+8, 0x1.1a81ep+9, 0x1.2be7e6p+8);
b.quadTo(0x1.199878p+9, 0x1.1e7142p+8, 0x1.196a1ap+9, 0x1.1a565ep+8);
b.lineTo(0x1.196a4ap+9, 0x1.1a5a7ap+8);
b.quadTo(0x1.1939cap+9, 0x1.162328p+8, 0x1.19270ep+9, 0x1.0fed9p+8);
b.lineTo(0x1.192718p+9, 0x1.0ff13ep+8);
b.quadTo(0x1.1912b4p+9, 0x1.0992bap+8, 0x1.1962c8p+9, 0x1.0073b8p+8);
b.lineTo(0x1.1962aap+9, 0x1.00772p+8);
b.quadTo(0x1.19b182p+9, 0x1.ee9d1ep+7, 0x1.1a8592p+9, 0x1.da3ed2p+7);
b.lineTo(0x1.1a854ep+9, 0x1.da4568p+7);
b.quadTo(0x1.1b57p+9, 0x1.c5f666p+7, 0x1.1c9cdep+9, 0x1.b12912p+7);
b.lineTo(0x1.1c9c7cp+9, 0x1.b12f5cp+7);
b.quadTo(0x1.1de512p+9, 0x1.9c1802p+7, 0x1.1fb65cp+9, 0x1.878b84p+7);
b.lineTo(0x1.1fb5d6p+9, 0x1.879172p+7);
b.quadTo(0x1.21930cp+9, 0x1.726916p+7, 0x1.23b84p+9, 0x1.6289ap+7);
b.lineTo(0x1.23b788p+9, 0x1.628ef6p+7);
b.quadTo(0x1.2621ap+9, 0x1.50a4c8p+7, 0x1.2972ep+9, 0x1.457ccap+7);
b.lineTo(0x1.2971d6p+9, 0x1.45804cp+7);
b.quadTo(0x1.2d282ep+9, 0x1.38fd68p+7, 0x1.33b808p+9, 0x1.3587b4p+7);
b.lineTo(0x1.33b6c2p+9, 0x1.35886p+7);
b.quadTo(0x1.39c13ap+9, 0x1.3252aep+7, 0x1.43bb32p+9, 0x1.3bb6b4p+7);
b.lineTo(0x1.43b9e4p+9, 0x1.3bb57ap+7);
b.quadTo(0x1.4d7bf8p+9, 0x1.44d9dap+7, 0x1.5b06dp+9, 0x1.61caf4p+7);
b.lineTo(0x1.5b059ap+9, 0x1.61c86p+7);
b.quadTo(0x1.689bf4p+9, 0x1.7ebef2p+7, 0x1.74b57ap+9, 0x1.ab563ep+7);
b.lineTo(0x1.74b47cp+9, 0x1.ab5292p+7);
b.quadTo(0x1.810cbap+9, 0x1.d8b846p+7, 0x1.86a248p+9, 0x1.0233d6p+8);
b.lineTo(0x1.86a1bp+9, 0x1.023188p+8);
b.quadTo(0x1.8c58f6p+9, 0x1.187eep+8, 0x1.8ca73ap+9, 0x1.2a5eccp+8);
b.quadTo(0x1.8cf8a8p+9, 0x1.3c751cp+8, 0x1.89bf96p+9, 0x1.4a7704p+8);
b.quadTo(0x1.8684d2p+9, 0x1.58899cp+8, 0x1.7f5726p+9, 0x1.63feacp+8);
b.lineTo(0x1.7f5836p+9, 0x1.63fcfap+8);
b.quadTo(0x1.788452p+9, 0x1.6ee8d2p+8, 0x1.6f18cp+9, 0x1.76a86ep+8);
b.lineTo(0x1.6f1a0cp+9, 0x1.76a75cp+8);
b.quadTo(0x1.65af3ep+9, 0x1.7e6c88p+8, 0x1.590bd8p+9, 0x1.817f0ep+8);
b.lineTo(0x1.590d4p+9, 0x1.817eb6p+8);
b.quadTo(0x1.4c9b52p+9, 0x1.848c6p+8, 0x1.3d73f2p+9, 0x1.82ec42p+8);
b.lineTo(0x1.3d755ap+9, 0x1.82ec68p+8);
b.quadTo(0x1.2e4e26p+9, 0x1.8154e2p+8, 0x1.206b7p+9, 0x1.7b8e64p+8);
b.lineTo(0x1.206cdp+9, 0x1.7b8ef6p+8);
b.quadTo(0x1.129154p+9, 0x1.75d396p+8, 0x1.05bf5ep+9, 0x1.6d3fe6p+8);
b.lineTo(0x1.05c0bp+9, 0x1.6d40c8p+8);
b.quadTo(0x1.f1edc2p+8, 0x1.64ba76p+8, 0x1.dda7dep+8, 0x1.5c07fp+8);
b.lineTo(0x1.ddaa64p+8, 0x1.5c0906p+8);
b.quadTo(0x1.c9388p+8, 0x1.534a4ap+8, 0x1.bc0c46p+8, 0x1.4bfa56p+8);
b.lineTo(0x1.bc0ea8p+8, 0x1.4bfbaap+8);
b.quadTo(0x1.aede6ep+8, 0x1.44ae34p+8, 0x1.a59b68p+8, 0x1.3e5b04p+8);
b.lineTo(0x1.a59da4p+8, 0x1.3e5c8ap+8);
b.quadTo(0x1.9c593ep+8, 0x1.380c18p+8, 0x1.8fbfccp+8, 0x1.2dca5p+8);
b.cubicTo(0x1.8fbfccp+8, 0x1.2dca5p+8, 0x1.8b7812p+8, 0x1.240084p+8, 0x1.8ad2aep+8, 0x1.240084p+8);
b.cubicTo(0x1.8e4ebp+8, 0x1.1fb8ccp+8, 0x1.91cab2p+8, 0x1.1acbaep+8, 0x1.98187cp+8, 0x1.1acbaep+8);
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek5(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.24d59p+9, 0x1.a35da6p+7);
b.quadTo(0x1.2fca0cp+9, 0x1.9dbdfcp+7, 0x1.3692d4p+9, 0x1.9d3edcp+7);
b.quadTo(0x1.3d559ep+9, 0x1.9cc02cp+7, 0x1.463532p+9, 0x1.9fa812p+7);
b.quadTo(0x1.4f2882p+9, 0x1.a2967p+7, 0x1.581af6p+9, 0x1.aa4f2ap+7);
b.quadTo(0x1.612508p+9, 0x1.b21c48p+7, 0x1.67c34cp+9, 0x1.bcb5dep+7);
b.quadTo(0x1.6e940ep+9, 0x1.c7a052p+7, 0x1.723cb6p+9, 0x1.d25eb6p+7);
b.quadTo(0x1.764fd4p+9, 0x1.de55b6p+7, 0x1.77cc3p+9, 0x1.e99a88p+7);
b.quadTo(0x1.798272p+9, 0x1.f6966ep+7, 0x1.7867bep+9, 0x1.0a3c56p+8);
b.cubicTo(0x1.7867bep+9, 0x1.0a3c56p+8, 0x1.78007p+9, 0x1.137916p+8, 0x1.75753p+9, 0x1.137916p+8);
b.cubicTo(0x1.72ba54p+9, 0x1.12aa7cp+8, 0x1.6fff7ap+9, 0x1.0cc56p+8, 0x1.6e1bf4p+9, 0x1.0cc56p+8);
b.quadTo(0x1.6f238ep+9, 0x1.fdad92p+7, 0x1.6ef3dp+9, 0x1.fc4378p+7);
b.quadTo(0x1.6e8a2cp+9, 0x1.f9224ap+7, 0x1.6c51cap+9, 0x1.f29d4ap+7);
b.quadTo(0x1.69aef2p+9, 0x1.eadfaep+7, 0x1.640bb4p+9, 0x1.e1d822p+7);
b.quadTo(0x1.5e35f8p+9, 0x1.d87fb8p+7, 0x1.55ff0ap+9, 0x1.d168d6p+7);
b.quadTo(0x1.4db07ep+9, 0x1.ca3d9p+7, 0x1.45644ep+9, 0x1.c785eep+7);
b.quadTo(0x1.3d0462p+9, 0x1.c4c7d4p+7, 0x1.36c1acp+9, 0x1.c53d24p+7);
b.quadTo(0x1.3084f4p+9, 0x1.c5b204p+7, 0x1.261b7p+9, 0x1.cb0a5ap+7);
b.cubicTo(0x1.261b7p+9, 0x1.cb0a5ap+7, 0x1.235ebap+9, 0x1.c4b28ep+7, 0x1.20dcd6p+9, 0x1.c4b28ep+7);
b.cubicTo(0x1.2082eap+9, 0x1.b9bfbcp+7, 0x1.2028fcp+9, 0x1.a4c55ap+7, 0x1.2218dcp+9, 0x1.a4c55ap+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.5e8198p+9, 0x1.66dacp+7);
b.quadTo(0x1.627c8ap+9, 0x1.93fcc4p+7, 0x1.662694p+9, 0x1.aa20cap+7);
b.quadTo(0x1.69f236p+9, 0x1.c10fd8p+7, 0x1.6ef93ep+9, 0x1.d740f6p+7);
b.quadTo(0x1.73fb8p+9, 0x1.ed5cfcp+7, 0x1.785fecp+9, 0x1.fb576p+7);
b.quadTo(0x1.7cb1e6p+9, 0x1.048b86p+8, 0x1.7fa99ap+9, 0x1.07951p+8);
b.quadTo(0x1.8292bap+9, 0x1.0a8facp+8, 0x1.8653f4p+9, 0x1.0c77b4p+8);
b.cubicTo(0x1.8653f4p+9, 0x1.0c77b4p+8, 0x1.8900aep+9, 0x1.13457p+8, 0x1.8a9f7ap+9, 0x1.13457p+8);
b.cubicTo(0x1.89f1a6p+9, 0x1.189ee6p+8, 0x1.8943d2p+9, 0x1.2135f2p+8, 0x1.868ac8p+9, 0x1.2135f2p+8);
b.quadTo(0x1.7f0546p+9, 0x1.1d6454p+8, 0x1.7b1b66p+9, 0x1.1962fp+8);
b.quadTo(0x1.77401ap+9, 0x1.15707ap+8, 0x1.722614p+9, 0x1.0d525p+8);
b.quadTo(0x1.6d1e8p+9, 0x1.055182p+8, 0x1.679042p+9, 0x1.f21d0ap+7);
b.quadTo(0x1.6206cap+9, 0x1.d9ac28p+7, 0x1.5dcfecp+9, 0x1.c03536p+7);
b.quadTo(0x1.597776p+9, 0x1.a5f33cp+7, 0x1.551368p+9, 0x1.74294p+7);
b.cubicTo(0x1.551368p+9, 0x1.74294p+7, 0x1.542864p+9, 0x1.5e51a8p+7, 0x1.558678p+9, 0x1.5e51a8p+7);
b.cubicTo(0x1.5820bp+9, 0x1.5aa59cp+7, 0x1.5abae8p+9, 0x1.5c71e4p+7, 0x1.5d9696p+9, 0x1.5c71e4p+7);
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek6(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.de007ep+8, 0x1.a9dep+7);
b.quadTo(0x1.f0ce8p+8, 0x1.a9d852p+7, 0x1.faa614p+8, 0x1.aa1c28p+7);
b.cubicTo(0x1.faa614p+8, 0x1.aa1c28p+7, 0x1.faabfcp+8, 0x1.aa1d86p+7, 0x1.fac84p+8, 0x1.aa1d86p+7);
b.quadTo(0x1.025b34p+9, 0x1.aab1ap+7, 0x1.07268ep+9, 0x1.ad2b5cp+7);
b.cubicTo(0x1.07268ep+9, 0x1.ad2b5cp+7, 0x1.06fffcp+9, 0x1.ad4e34p+7, 0x1.076334p+9, 0x1.ad4e34p+7);
b.quadTo(0x1.0c0c2ep+9, 0x1.b00b36p+7, 0x1.102e3cp+9, 0x1.b5ac4ap+7);
b.cubicTo(0x1.102e3cp+9, 0x1.b5ac4ap+7, 0x1.1026b6p+9, 0x1.b5d6dp+7, 0x1.104c4ap+9, 0x1.b5d6dp+7);
b.quadTo(0x1.141d7cp+9, 0x1.bb5be4p+7, 0x1.177542p+9, 0x1.c26886p+7);
b.cubicTo(0x1.177542p+9, 0x1.c26886p+7, 0x1.176fe6p+9, 0x1.c2a7d8p+7, 0x1.179264p+9, 0x1.c2a7d8p+7);
b.quadTo(0x1.1aa7d8p+9, 0x1.c97f0cp+7, 0x1.1d38acp+9, 0x1.d0c226p+7);
b.cubicTo(0x1.1d38acp+9, 0x1.d0c226p+7, 0x1.1d415ep+9, 0x1.d0f2dp+7, 0x1.1d49a6p+9, 0x1.d0f2dp+7);
b.quadTo(0x1.1fbdc6p+9, 0x1.d84348p+7, 0x1.2151a4p+9, 0x1.de37c8p+7);
b.cubicTo(0x1.2151a4p+9, 0x1.de37c8p+7, 0x1.2155eap+9, 0x1.de8b02p+7, 0x1.216732p+9, 0x1.de8b02p+7);
b.quadTo(0x1.23144p+9, 0x1.e53becp+7, 0x1.257e36p+9, 0x1.f49234p+7);
b.cubicTo(0x1.257e36p+9, 0x1.f49234p+7, 0x1.26f696p+9, 0x1.0522aap+8, 0x1.2642cap+9, 0x1.0522aap+8);
b.cubicTo(0x1.23ecf4p+9, 0x1.08136cp+8, 0x1.21971cp+9, 0x1.099c94p+8, 0x1.1e802ap+9, 0x1.099c94p+8);
b.quadTo(0x1.1b379p+9, 0x1.fe5c48p+7, 0x1.1a5ae8p+9, 0x1.faeb58p+7);
b.lineTo(0x1.1a74ap+9, 0x1.fb4f3p+7);
b.quadTo(0x1.194cdap+9, 0x1.f6f2b4p+7, 0x1.17587p+9, 0x1.f11ef6p+7);
b.lineTo(0x1.1771f4p+9, 0x1.f1691ap+7);
b.quadTo(0x1.1553ap+9, 0x1.eb6a14p+7, 0x1.12b38cp+9, 0x1.e5973cp+7);
b.lineTo(0x1.12cb62p+9, 0x1.e5cacep+7);
b.quadTo(0x1.10106cp+9, 0x1.e008d8p+7, 0x1.0cde78p+9, 0x1.db6a0ap+7);
b.lineTo(0x1.0cf51p+9, 0x1.db89c2p+7);
b.quadTo(0x1.09b08ep+9, 0x1.d7166ap+7, 0x1.05c8a6p+9, 0x1.d4cadcp+7);
b.lineTo(0x1.05ded2p+9, 0x1.d4d718p+7);
b.quadTo(0x1.01919p+9, 0x1.d29e78p+7, 0x1.fa3914p+8, 0x1.d21986p+7);
b.lineTo(0x1.fa6128p+8, 0x1.d21b3cp+7);
b.quadTo(0x1.f0ad8cp+8, 0x1.d1d85ep+7, 0x1.de0382p+8, 0x1.d1dep+7);
b.cubicTo(0x1.de0382p+8, 0x1.d1dep+7, 0x1.d87e9cp+8, 0x1.c8ead2p+7, 0x1.d402d6p+8, 0x1.c8ead2p+7);
b.cubicTo(0x1.d402p+8, 0x1.bde104p+7, 0x1.d4012ap+8, 0x1.a9dfaap+7, 0x1.d87b96p+8, 0x1.a9dfaap+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.fd63fap+8, 0x1.c765b6p+7);
b.quadTo(0x1.04860ap+9, 0x1.df4834p+7, 0x1.05e296p+9, 0x1.e308eep+7);
b.lineTo(0x1.057df6p+9, 0x1.e20e3cp+7);
b.quadTo(0x1.06feep+9, 0x1.e56ce4p+7, 0x1.090ffcp+9, 0x1.e8f05ap+7);
b.lineTo(0x1.088b38p+9, 0x1.e8307cp+7);
b.quadTo(0x1.0a4dc6p+9, 0x1.ea4d46p+7, 0x1.0c7b92p+9, 0x1.eafcd8p+7);
b.lineTo(0x1.0bdbe6p+9, 0x1.eaf28ep+7);
b.quadTo(0x1.0d7acep+9, 0x1.eaa5a2p+7, 0x1.0eb7cap+9, 0x1.e850a6p+7);
b.lineTo(0x1.0e2fdap+9, 0x1.e97a22p+7);
b.quadTo(0x1.0fb6d6p+9, 0x1.e5a316p+7, 0x1.11107cp+9, 0x1.e02edap+7);
b.lineTo(0x1.10b854p+9, 0x1.e1b9dcp+7);
b.quadTo(0x1.11f874p+9, 0x1.db80bcp+7, 0x1.12e024p+9, 0x1.d400d4p+7);
b.lineTo(0x1.12b7fap+9, 0x1.d5720ap+7);
b.quadTo(0x1.13b8c6p+9, 0x1.cb221p+7, 0x1.14f9p+9, 0x1.be6878p+7);
b.lineTo(0x1.14d6ap+9, 0x1.bff53p+7);
b.quadTo(0x1.15de1p+9, 0x1.b24b5cp+7, 0x1.173afcp+9, 0x1.a0b11ap+7);
b.cubicTo(0x1.173afcp+9, 0x1.a0b11ap+7, 0x1.180bfcp+9, 0x1.903d0ep+7, 0x1.1ad98cp+9, 0x1.903d0ep+7);
b.cubicTo(0x1.1d7c6p+9, 0x1.938114p+7, 0x1.201f32p+9, 0x1.a1fb58p+7, 0x1.219962p+9, 0x1.a1fb58p+7);
b.quadTo(0x1.1f6e64p+9, 0x1.bdfb0cp+7, 0x1.1e69dp+9, 0x1.cb7efp+7);
b.cubicTo(0x1.1e69dp+9, 0x1.cb7efp+7, 0x1.1e5fdep+9, 0x1.cc8b5ep+7, 0x1.1e540cp+9, 0x1.cc8b5ep+7);
b.quadTo(0x1.1d08e2p+9, 0x1.d9b43p+7, 0x1.1c09c2p+9, 0x1.e3f316p+7);
b.cubicTo(0x1.1c09c2p+9, 0x1.e3f316p+7, 0x1.1bfd9cp+9, 0x1.e4eb9cp+7, 0x1.1bf028p+9, 0x1.e4eb9cp+7);
b.quadTo(0x1.1a7aep+9, 0x1.f100d4p+7, 0x1.1883bp+9, 0x1.fac904p+7);
b.cubicTo(0x1.1883bp+9, 0x1.fac904p+7, 0x1.186816p+9, 0x1.fbd6cep+7, 0x1.184a88p+9, 0x1.fbd6cep+7);
b.quadTo(0x1.16029ap+9, 0x1.0286c6p+8, 0x1.138236p+9, 0x1.05abd6p+8);
b.cubicTo(0x1.138236p+9, 0x1.05abd6p+8, 0x1.1356a8p+9, 0x1.0614cp+8, 0x1.1328d2p+9, 0x1.0614cp+8);
b.quadTo(0x1.0fef5ap+9, 0x1.091e0cp+8, 0x1.0c526ep+9, 0x1.0973cap+8);
b.cubicTo(0x1.0c526ep+9, 0x1.0973cap+8, 0x1.0c1d34p+9, 0x1.0977p+8, 0x1.0be7ep+9, 0x1.0977p+8);
b.quadTo(0x1.087422p+9, 0x1.08ebecp+8, 0x1.05ab74p+9, 0x1.07403ap+8);
b.cubicTo(0x1.05ab74p+9, 0x1.07403ap+8, 0x1.057e3p+9, 0x1.070544p+8, 0x1.05523p+9, 0x1.070544p+8);
b.quadTo(0x1.029538p+9, 0x1.04b176p+8, 0x1.009aa2p+9, 0x1.0279cap+8);
b.cubicTo(0x1.009aa2p+9, 0x1.0279cap+8, 0x1.007852p+9, 0x1.022914p+8, 0x1.005666p+9, 0x1.022914p+8);
b.quadTo(0x1.fc185ep+8, 0x1.fe025cp+7, 0x1.ef1406p+8, 0x1.e3564ap+7);
b.cubicTo(0x1.ef1406p+8, 0x1.e3564ap+7, 0x1.eb391ep+8, 0x1.cec3dap+7, 0x1.eb4cc2p+8, 0x1.cec3dap+7);
b.cubicTo(0x1.ef3fdcp+8, 0x1.c70e0cp+7, 0x1.f332f4p+8, 0x1.bf7f86p+7, 0x1.f98914p+8, 0x1.bf7f86p+7);
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek7(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.35e382p+7, 0x1.4b157p+6);
b.quadTo(0x1.7000c8p+7, 0x1.636ce6p+6, 0x1.b110d6p+7, 0x1.98653p+6);
b.cubicTo(0x1.b110d6p+7, 0x1.98653p+6, 0x1.b150fep+7, 0x1.98e198p+6, 0x1.b1a44p+7, 0x1.98e198p+6);
b.quadTo(0x1.03ddacp+8, 0x1.e82182p+6, 0x1.32e5a4p+8, 0x1.2383bp+7);
b.cubicTo(0x1.32e5a4p+8, 0x1.2383bp+7, 0x1.322e14p+8, 0x1.24642ep+7, 0x1.33bcecp+8, 0x1.24642ep+7);
b.quadTo(0x1.6aa9b6p+8, 0x1.5dd264p+7, 0x1.ab3e12p+8, 0x1.a23452p+7);
b.cubicTo(0x1.ab3e12p+8, 0x1.a23452p+7, 0x1.b01ec2p+8, 0x1.b37b92p+7, 0x1.b1fbccp+8, 0x1.b37b92p+7);
b.cubicTo(0x1.af66bp+8, 0x1.bd3cf4p+7, 0x1.acd192p+8, 0x1.cab868p+7, 0x1.a6c30ep+8, 0x1.cab868p+7);
b.quadTo(0x1.6159c6p+8, 0x1.8138b4p+7, 0x1.29c294p+8, 0x1.471852p+7);
b.lineTo(0x1.29e3acp+8, 0x1.473a5p+7);
b.quadTo(0x1.f65b3ep+7, 0x1.181a06p+7, 0x1.a129bcp+7, 0x1.e1c77cp+6);
b.lineTo(0x1.a1fc12p+7, 0x1.e27ddp+6);
b.quadTo(0x1.644498p+7, 0x1.b03ee2p+6, 0x1.2db07ep+7, 0x1.99629p+6);
b.cubicTo(0x1.2db07ep+7, 0x1.99629p+6, 0x1.22e2b2p+7, 0x1.7fa496p+6, 0x1.1bf37cp+7, 0x1.7fa496p+6);
b.cubicTo(0x1.1e36b8p+7, 0x1.6a08fcp+6, 0x1.2079f4p+7, 0x1.468ef6p+6, 0x1.2b15b4p+7, 0x1.468ef6p+6);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.05afa8p+8, 0x1.d2b1d4p+6);
b.quadTo(0x1.ff9c24p+7, 0x1.0cc8d6p+7, 0x1.ff5dc4p+7, 0x1.23ff1ep+7);
b.lineTo(0x1.fe94b6p+7, 0x1.1e3bb4p+7);
b.quadTo(0x1.025d6p+8, 0x1.33817ep+7, 0x1.0664b8p+8, 0x1.40ef6ep+7);
b.lineTo(0x1.04fa7p+8, 0x1.3d433ap+7);
b.quadTo(0x1.0a9a82p+8, 0x1.48cd18p+7, 0x1.1392bep+8, 0x1.527398p+7);
b.lineTo(0x1.11c7bap+8, 0x1.50f38ep+7);
b.quadTo(0x1.19ce54p+8, 0x1.55e59ap+7, 0x1.1ceedap+8, 0x1.519804p+7);
b.lineTo(0x1.1b1f1ep+8, 0x1.54cbc4p+7);
b.quadTo(0x1.1ca31p+8, 0x1.516164p+7, 0x1.1cb704p+8, 0x1.502154p+7);
b.quadTo(0x1.1caa7cp+8, 0x1.50ea52p+7, 0x1.1cca4p+8, 0x1.51650ap+7);
b.lineTo(0x1.1e29a6p+8, 0x1.55691p+7);
b.quadTo(0x1.1a2aep+8, 0x1.4c542cp+7, 0x1.07b6e2p+8, 0x1.3d8ab6p+7);
b.cubicTo(0x1.07b6e2p+8, 0x1.3d8ab6p+7, 0x1.029756p+8, 0x1.2dc8dcp+7, 0x1.001928p+8, 0x1.2dc8dcp+7);
b.cubicTo(0x1.0226a4p+8, 0x1.2389c4p+7, 0x1.043422p+8, 0x1.144e5p+7, 0x1.0a0792p+8, 0x1.144e5p+7);
b.quadTo(0x1.260864p+8, 0x1.2abedcp+7, 0x1.2d2d9ap+8, 0x1.3afcecp+7);
b.cubicTo(0x1.2d2d9ap+8, 0x1.3afcecp+7, 0x1.2db726p+8, 0x1.3d8f22p+7, 0x1.2e2d44p+8, 0x1.3d8f22p+7);
b.quadTo(0x1.3147b4p+8, 0x1.498bfep+7, 0x1.308fbp+8, 0x1.55141p+7);
b.quadTo(0x1.2fb734p+8, 0x1.62a53cp+7, 0x1.2a14c2p+8, 0x1.6f5854p+7);
b.cubicTo(0x1.2a14c2p+8, 0x1.6f5854p+7, 0x1.298b5cp+8, 0x1.71a166p+7, 0x1.28ef9p+8, 0x1.71a166p+7);
b.quadTo(0x1.1d2afcp+8, 0x1.81d2f2p+7, 0x1.0be42ep+8, 0x1.772d9ap+7);
b.cubicTo(0x1.0be42ep+8, 0x1.772d9ap+7, 0x1.0b4562p+8, 0x1.764b02p+7, 0x1.0aab8p+8, 0x1.764b02p+7);
b.quadTo(0x1.fcda2ep+7, 0x1.691ebcp+7, 0x1.ed512ep+7, 0x1.592feep+7);
b.cubicTo(0x1.ed512ep+7, 0x1.592feep+7, 0x1.ec3bfap+7, 0x1.56d836p+7, 0x1.eb48e6p+7, 0x1.56d836p+7);
b.quadTo(0x1.dfbfc2p+7, 0x1.439e1ep+7, 0x1.d82762p+7, 0x1.29570cp+7);
b.cubicTo(0x1.d82762p+7, 0x1.29570cp+7, 0x1.d79cd4p+7, 0x1.258698p+7, 0x1.d75918p+7, 0x1.258698p+7);
b.quadTo(0x1.d7ad8p+7, 0x1.061d7ep+7, 0x1.e568bp+7, 0x1.b97e2cp+6);
b.cubicTo(0x1.e568bp+7, 0x1.b97e2cp+6, 0x1.e8e2ep+7, 0x1.992cfep+6, 0x1.f436fcp+7, 0x1.992cfep+6);
b.cubicTo(0x1.feb0eap+7, 0x1.a0216p+6, 0x1.04956cp+8, 0x1.bdbdf8p+6, 0x1.076ccp+8, 0x1.bdbdf8p+6);
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek8(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.1d815ep+9, 0x1.2ac52cp+7);
b.quadTo(0x1.19d00cp+9, 0x1.2c49ccp+7, 0x1.169b82p+9, 0x1.2f4974p+7);
b.quadTo(0x1.13ad6p+9, 0x1.32074p+7, 0x1.d19f4ep+8, 0x1.88edf2p+7);
b.quadTo(0x1.7b9e44p+8, 0x1.e01b3p+7, 0x1.644cf6p+8, 0x1.01b5ep+8);
b.quadTo(0x1.4d6f54p+8, 0x1.13068cp+8, 0x1.42a842p+8, 0x1.21c008p+8);
b.quadTo(0x1.393d4ep+8, 0x1.2e9deap+8, 0x1.3a3cf4p+8, 0x1.39daeap+8);
b.quadTo(0x1.3b4abp+8, 0x1.45b662p+8, 0x1.460926p+8, 0x1.52848cp+8);
b.quadTo(0x1.51b41ep+8, 0x1.606c9ap+8, 0x1.635858p+8, 0x1.6add6ap+8);
b.quadTo(0x1.75a35ep+8, 0x1.75b0fp+8, 0x1.868846p+8, 0x1.7bc22ap+8);
b.quadTo(0x1.97b05p+8, 0x1.81eb8p+8, 0x1.a4f588p+8, 0x1.84c188p+8);
b.quadTo(0x1.b294d8p+8, 0x1.87aad4p+8, 0x1.bc6e46p+8, 0x1.892306p+8);
b.quadTo(0x1.c6e4c4p+8, 0x1.8ab2aap+8, 0x1.d03e64p+8, 0x1.8cb014p+8);
b.quadTo(0x1.d9eb0cp+8, 0x1.8ebf2ap+8, 0x1.e8c88ep+8, 0x1.93f66p+8);
b.cubicTo(0x1.e8c88ep+8, 0x1.93f66p+8, 0x1.edfdc6p+8, 0x1.9b8056p+8, 0x1.f0bc68p+8, 0x1.9b8056p+8);
b.cubicTo(0x1.eee8ap+8, 0x1.a0b58ep+8, 0x1.ed14dap+8, 0x1.a8a968p+8, 0x1.e75eaap+8, 0x1.a8a968p+8);
b.quadTo(0x1.d480f4p+8, 0x1.a20ad6p+8, 0x1.cc149cp+8, 0x1.a03fecp+8);
b.quadTo(0x1.c3553cp+8, 0x1.9e6356p+8, 0x1.b97abap+8, 0x1.9ceafap+8);
b.quadTo(0x1.af0328p+8, 0x1.9b5b2cp+8, 0x1.a0c778p+8, 0x1.985078p+8);
b.quadTo(0x1.9231bp+8, 0x1.95328p+8, 0x1.7fc5bap+8, 0x1.8e94d6p+8);
b.quadTo(0x1.6d16a2p+8, 0x1.87df1p+8, 0x1.5928a8p+8, 0x1.7c1396p+8);
b.quadTo(0x1.4493e2p+8, 0x1.6fe566p+8, 0x1.36b6dap+8, 0x1.5f5f74p+8);
b.quadTo(0x1.27ed5p+8, 0x1.4dbf9ep+8, 0x1.26510cp+8, 0x1.3ba016p+8);
b.quadTo(0x1.24a6b2p+8, 0x1.28e216p+8, 0x1.3284bep+8, 0x1.15eff8p+8);
b.quadTo(0x1.3f06acp+8, 0x1.04d974p+8, 0x1.583a0ap+8, 0x1.e38842p+7);
b.quadTo(0x1.70f9bcp+8, 0x1.be0cdp+7, 0x1.c894b2p+8, 0x1.65400ep+7);
b.quadTo(0x1.103aap+9, 0x1.0c2ccp+7, 0x1.14547ep+9, 0x1.08568cp+7);
b.quadTo(0x1.1827f4p+9, 0x1.04c234p+7, 0x1.1c7ba2p+9, 0x1.02fad4p+7);
b.cubicTo(0x1.1c7ba2p+9, 0x1.02fad4p+7, 0x1.1f3a62p+9, 0x1.09d98cp+7, 0x1.21af92p+9, 0x1.09d98cp+7);
b.cubicTo(0x1.21f7cap+9, 0x1.14d48ap+7, 0x1.224004p+9, 0x1.29a444p+7, 0x1.20401ep+9, 0x1.29a444p+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.679a9cp+8, 0x1.7643bap+6);
b.quadTo(0x1.760ddp+8, 0x1.b57adcp+6, 0x1.83884ep+8, 0x1.e55d5ap+6);
b.quadTo(0x1.914dep+8, 0x1.0b254ap+7, 0x1.a3e0e8p+8, 0x1.2a7b28p+7);
b.quadTo(0x1.b6b87ap+8, 0x1.4a44aap+7, 0x1.c61adep+8, 0x1.66a8fap+7);
b.quadTo(0x1.d5f2fap+8, 0x1.83e688p+7, 0x1.dea5fep+8, 0x1.9a7da6p+7);
b.quadTo(0x1.e7fceep+8, 0x1.b2be72p+7, 0x1.eae64p+8, 0x1.c6e6ecp+7);
b.quadTo(0x1.ee362p+8, 0x1.ddd57p+7, 0x1.ea2dd4p+8, 0x1.f3053ap+7);
b.quadTo(0x1.e65a72p+8, 0x1.038f88p+8, 0x1.da8e6p+8, 0x1.0ce078p+8);
b.quadTo(0x1.cfa38ep+8, 0x1.157f88p+8, 0x1.bc1074p+8, 0x1.1c7fdap+8);
b.quadTo(0x1.a9773ep+8, 0x1.2326c8p+8, 0x1.90a8dap+8, 0x1.28c7eap+8);
b.quadTo(0x1.78bbcp+8, 0x1.2e35eap+8, 0x1.66047ep+8, 0x1.33a7f6p+8);
b.quadTo(0x1.54e3bcp+8, 0x1.38a3bcp+8, 0x1.4f8492p+8, 0x1.3ce306p+8);
b.quadTo(0x1.4cdc84p+8, 0x1.3efcb6p+8, 0x1.4c45d8p+8, 0x1.4032d2p+8);
b.quadTo(0x1.4c6564p+8, 0x1.3ff1e4p+8, 0x1.4c65ccp+8, 0x1.3f6296p+8);
b.quadTo(0x1.4c649ap+8, 0x1.410944p+8, 0x1.4dc2b4p+8, 0x1.42f66p+8);
b.quadTo(0x1.500d82p+8, 0x1.4630ep+8, 0x1.5511aep+8, 0x1.49e26cp+8);
b.quadTo(0x1.5a9a32p+8, 0x1.4df568p+8, 0x1.63dd5cp+8, 0x1.525828p+8);
b.quadTo(0x1.6da656p+8, 0x1.56fa44p+8, 0x1.7b34cep+8, 0x1.5c27ecp+8);
b.quadTo(0x1.89117ap+8, 0x1.617374p+8, 0x1.98fa98p+8, 0x1.674fa6p+8);
b.quadTo(0x1.a8daeap+8, 0x1.6d289ep+8, 0x1.b8c512p+8, 0x1.72ac86p+8);
b.quadTo(0x1.c86eb8p+8, 0x1.781a14p+8, 0x1.e3dae6p+8, 0x1.7f6d16p+8);
b.cubicTo(0x1.e3dae6p+8, 0x1.7f6d16p+8, 0x1.e92ff2p+8, 0x1.8655eep+8, 0x1.ec5c32p+8, 0x1.8655eep+8);
b.cubicTo(0x1.eaef9cp+8, 0x1.8baafcp+8, 0x1.e98304p+8, 0x1.942c48p+8, 0x1.e406c4p+8, 0x1.942c48p+8);
b.quadTo(0x1.c291e4p+8, 0x1.8b3cb4p+8, 0x1.b2388ap+8, 0x1.859242p+8);
b.quadTo(0x1.a21fb2p+8, 0x1.7ffe2ap+8, 0x1.921104p+8, 0x1.7a1422p+8);
b.quadTo(0x1.820b22p+8, 0x1.742d54p+8, 0x1.7411cep+8, 0x1.6ed6dcp+8);
b.quadTo(0x1.65ca46p+8, 0x1.696284p+8, 0x1.5b4e4p+8, 0x1.646bap+8);
b.quadTo(0x1.504c6ap+8, 0x1.5f356p+8, 0x1.4935eep+8, 0x1.59fd5cp+8);
b.quadTo(0x1.419b1ap+8, 0x1.5463e8p+8, 0x1.3d73e8p+8, 0x1.4e8a68p+8);
b.quadTo(0x1.386002p+8, 0x1.476384p+8, 0x1.3865dp+8, 0x1.3f5432p+8);
b.quadTo(0x1.3868b4p+8, 0x1.3b510cp+8, 0x1.3a48acp+8, 0x1.37753ap+8);
b.quadTo(0x1.3cdeep+8, 0x1.32225ep+8, 0x1.431d0ap+8, 0x1.2d32c2p+8);
b.quadTo(0x1.4b8cep+8, 0x1.26870cp+8, 0x1.606e1ep+8, 0x1.2073d2p+8);
b.quadTo(0x1.73b8dcp+8, 0x1.1ad6dep+8, 0x1.8c3bc2p+8, 0x1.1546dep+8);
b.quadTo(0x1.a3dd5ep+8, 0x1.0feap+8, 0x1.b55428p+8, 0x1.09aaeep+8);
b.quadTo(0x1.c5d10ep+8, 0x1.03c54p+8, 0x1.ce293cp+8, 0x1.fa5c9ep+7);
b.quadTo(0x1.d5a02ap+8, 0x1.ee9282p+7, 0x1.d77cc8p+8, 0x1.e4ca56p+7);
b.quadTo(0x1.d9247cp+8, 0x1.dc182p+7, 0x1.d7af5cp+8, 0x1.d200a4p+7);
b.quadTo(0x1.d5d3aep+8, 0x1.c5231ep+7, 0x1.cecd9ep+8, 0x1.b2e5eap+7);
b.quadTo(0x1.c723a2p+8, 0x1.9eff08p+7, 0x1.b88abep+8, 0x1.840e96p+7);
b.quadTo(0x1.a97c22p+8, 0x1.6844e6p+7, 0x1.96fbb4p+8, 0x1.490e68p+7);
b.quadTo(0x1.8436bcp+8, 0x1.296446p+7, 0x1.76404ep+8, 0x1.1096e2p+7);
b.quadTo(0x1.67feccp+8, 0x1.ee8844p+6, 0x1.58d8p+8, 0x1.ac3f66p+6);
b.cubicTo(0x1.58d8p+8, 0x1.ac3f66p+6, 0x1.551e96p+8, 0x1.82a1f8p+6, 0x1.55671cp+8, 0x1.82a1f8p+6);
b.cubicTo(0x1.5979d8p+8, 0x1.73bc54p+6, 0x1.5d8c96p+8, 0x1.65f8c6p+6, 0x1.63e134p+8, 0x1.65f8c6p+6);
b.close();
    testPathOp(reporter, left, b, kIntersect_SkPathOp, filename);
}

static void pentrek9(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.357ca2p+8, 0x1.9ca62cp+6);
b.lineTo(0x1.a2f942p+7, 0x1.93298cp+8);
b.cubicTo(0x1.a2f942p+7, 0x1.93298cp+8, 0x1.9f7baap+7, 0x1.9b3b6ep+8, 0x1.9425b6p+7, 0x1.9b3b6ep+8);
b.cubicTo(0x1.89aceap+7, 0x1.997ca2p+8, 0x1.7f341ep+7, 0x1.9212dap+8, 0x1.798924p+7, 0x1.9212dap+8);
b.lineTo(0x1.22835ep+8, 0x1.8359d4p+6);
b.cubicTo(0x1.22835ep+8, 0x1.8359d4p+6, 0x1.24422cp+8, 0x1.63124ap+6, 0x1.29ed26p+8, 0x1.63124ap+6);
b.cubicTo(0x1.2f298cp+8, 0x1.6a0d7cp+6, 0x1.3465fp+8, 0x1.87b496p+6, 0x1.373b6ep+8, 0x1.87b496p+6);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.2e5f28p+8, 0x1.93298cp+6);
b.lineTo(0x1.94be5p+7, 0x1.90ca62p+8);
b.cubicTo(0x1.94be5p+7, 0x1.90ca62p+8, 0x1.93deeap+7, 0x1.92cedcp+8, 0x1.91096ep+7, 0x1.92cedcp+8);
b.cubicTo(0x1.8e6b3ap+7, 0x1.925f28p+8, 0x1.8bcd08p+7, 0x1.9084b6p+8, 0x1.8a624ap+7, 0x1.9084b6p+8);
b.lineTo(0x1.29a0d8p+8, 0x1.8cd674p+6);
b.cubicTo(0x1.29a0d8p+8, 0x1.8cd674p+6, 0x1.2a108ap+8, 0x1.84c492p+6, 0x1.2b7b4ap+8, 0x1.84c492p+6);
b.cubicTo(0x1.2cca62p+8, 0x1.86835ep+6, 0x1.2e197cp+8, 0x1.8ded26p+6, 0x1.2ecedcp+8, 0x1.8ded26p+6);
b.close();
    testPathOp(reporter, left, b, kDifference_SkPathOp, filename);
}

static void pentrek10(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.ad3f6p+8, 0x1.c2fbe8p+7);
b.quadTo(0x1.a88b66p+8, 0x1.d22bcp+7, 0x1.a6ef74p+8, 0x1.d997bp+7);
b.lineTo(0x1.a6f15cp+8, 0x1.d98edap+7);
b.quadTo(0x1.a56b24p+8, 0x1.e09fe2p+7, 0x1.a409f6p+8, 0x1.eccee2p+7);
b.lineTo(0x1.a40b04p+8, 0x1.ecc582p+7);
b.quadTo(0x1.a2b32cp+8, 0x1.f8bcbcp+7, 0x1.a29296p+8, 0x1.05defap+8);
b.lineTo(0x1.a292a4p+8, 0x1.05da2ep+8);
b.quadTo(0x1.a275e8p+8, 0x1.0f9874p+8, 0x1.a40b5cp+8, 0x1.1a2da8p+8);
b.lineTo(0x1.a40aa6p+8, 0x1.1a28ecp+8);
b.quadTo(0x1.a5a082p+8, 0x1.249ea8p+8, 0x1.a8e43ap+8, 0x1.2dff74p+8);
b.lineTo(0x1.a8e2a8p+8, 0x1.2dfaf2p+8);
b.quadTo(0x1.ac2edap+8, 0x1.37659ep+8, 0x1.b0cd6p+8, 0x1.3f9bd6p+8);
b.lineTo(0x1.b0cb06p+8, 0x1.3f97acp+8);
b.quadTo(0x1.b558eep+8, 0x1.47a756p+8, 0x1.baa71p+8, 0x1.4dac02p+8);
b.lineTo(0x1.baa3ecp+8, 0x1.4da872p+8);
b.quadTo(0x1.c00c1cp+8, 0x1.53c4dp+8, 0x1.c5e4c8p+8, 0x1.585a38p+8);
b.lineTo(0x1.c5e10cp+8, 0x1.58574ep+8);
b.quadTo(0x1.cb9b8ep+8, 0x1.5cd0bp+8, 0x1.d73b8ep+8, 0x1.626c6p+8);
b.lineTo(0x1.d73748p+8, 0x1.626a52p+8);
b.quadTo(0x1.e327fcp+8, 0x1.6825f6p+8, 0x1.efafa4p+8, 0x1.6c0854p+8);
b.lineTo(0x1.efab1cp+8, 0x1.6c06eep+8);
b.quadTo(0x1.fafda6p+8, 0x1.6f8396p+8, 0x1.006ed4p+9, 0x1.6eefbcp+8);
b.lineTo(0x1.006c7ep+9, 0x1.6ef032p+8);
b.quadTo(0x1.03c278p+9, 0x1.6e4524p+8, 0x1.0630b8p+9, 0x1.6c8bfap+8);
b.lineTo(0x1.062e86p+9, 0x1.6c8d88p+8);
b.quadTo(0x1.07ebecp+9, 0x1.6b4fecp+8, 0x1.08fe0ap+9, 0x1.68cc5ap+8);
b.lineTo(0x1.08fc88p+9, 0x1.68cfe4p+8);
b.quadTo(0x1.09d31p+9, 0x1.66d65ep+8, 0x1.09f092p+9, 0x1.624392p+8);
b.lineTo(0x1.09f074p+9, 0x1.624848p+8);
b.quadTo(0x1.0a0cfcp+9, 0x1.5db13cp+8, 0x1.06cd2ep+9, 0x1.543ceep+8);
b.lineTo(0x1.06ce8ap+9, 0x1.5440ep+8);
b.quadTo(0x1.02fb2p+9, 0x1.492a7cp+8, 0x1.f34ab4p+8, 0x1.39a94ep+8);
b.lineTo(0x1.f34e66p+8, 0x1.39ac5cp+8);
b.quadTo(0x1.e0410ap+8, 0x1.29e95cp+8, 0x1.c74398p+8, 0x1.1c0918p+8);
b.lineTo(0x1.c747c6p+8, 0x1.1c0b6ap+8);
b.quadTo(0x1.ae3194p+8, 0x1.0e2d16p+8, 0x1.9b13aep+8, 0x1.078254p+8);
b.lineTo(0x1.9b1832p+8, 0x1.0783e6p+8);
b.quadTo(0x1.887f88p+8, 0x1.011194p+8, 0x1.7f685ap+8, 0x1.009862p+8);
b.lineTo(0x1.7f6d22p+8, 0x1.0098ap+8);
b.quadTo(0x1.772afcp+8, 0x1.002e7cp+8, 0x1.73569cp+8, 0x1.01c534p+8);
b.lineTo(0x1.735b04p+8, 0x1.01c36p+8);
b.quadTo(0x1.6e82p+8, 0x1.03c8fp+8, 0x1.6a0082p+8, 0x1.07075cp+8);
b.lineTo(0x1.6a045ep+8, 0x1.070494p+8);
b.quadTo(0x1.6456d4p+8, 0x1.0b1f1cp+8, 0x1.5d176cp+8, 0x1.0fb828p+8);
b.lineTo(0x1.5d1b72p+8, 0x1.0fb59ap+8);
b.quadTo(0x1.55f3bep+8, 0x1.144464p+8, 0x1.49bf94p+8, 0x1.1b5e52p+8);
b.cubicTo(0x1.49bf94p+8, 0x1.1b5e52p+8, 0x1.44fa6p+8, 0x1.1c8662p+8, 0x1.3ed9fep+8, 0x1.1c8662p+8);
b.cubicTo(0x1.3c1362p+8, 0x1.17c13p+8, 0x1.394cc8p+8, 0x1.0cdb9ap+8, 0x1.3aeb52p+8, 0x1.0cdb9ap+8);
b.quadTo(0x1.4b8bc2p+8, 0x1.032edp+8, 0x1.525cb2p+8, 0x1.fdae96p+7);
b.cubicTo(0x1.525cb2p+8, 0x1.fdae96p+7, 0x1.473864p+8, 0x1.ef8244p+7, 0x1.5d8866p+8, 0x1.ef8244p+7);
b.quadTo(0x1.591d48p+8, 0x1.f51d66p+7, 0x1.5e4d3ap+8, 0x1.ed9ddcp+7);
b.cubicTo(0x1.5e4d3ap+8, 0x1.ed9ddcp+7, 0x1.15cefcp+9, 0x1.8ace84p+8, 0x1.2173bep+7, 0x1.8ace84p+8);
b.quadTo(0x1.64b024p+8, 0x1.e46bd6p+7, 0x1.6ba84p+8, 0x1.de9bb8p+7);
b.cubicTo(0x1.6ba84p+8, 0x1.de9bb8p+7, 0x1.52b93ep+8, 0x1.c9e3d2p+7, 0x1.84a09ep+8, 0x1.c9e3d2p+7);
b.quadTo(0x1.73ab0cp+8, 0x1.d7f61ep+7, 0x1.806ddap+8, 0x1.d93e2p+7);
b.cubicTo(0x1.806ddap+8, 0x1.d93e2p+7, 0x1.3fdbp+5, 0x1.fd1e16p+7, 0x1.6c6e56p+9, 0x1.fd1e16p+7);
b.quadTo(0x1.8c5ebp+8, 0x1.da7c82p+7, 0x1.a1a52ep+8, 0x1.e93ca4p+7);
b.cubicTo(0x1.a1a52ep+8, 0x1.e93ca4p+7, -0x1.4d6368p+7, 0x1.c0483p+8, 0x1.f4cfb2p+9, 0x1.c0483p+8);
b.quadTo(0x1.b66228p+8, 0x1.f7b3acp+7, 0x1.d0f4e6p+8, 0x1.0a8a8ap+8);
b.cubicTo(0x1.d0f4e6p+8, 0x1.0a8a8ap+8, 0x1.691b1p+7, 0x1.a84e4p+8, 0x1.768a92p+9, 0x1.a84e4p+8);
b.quadTo(0x1.eb942ap+8, 0x1.1952ccp+8, 0x1.000708p+9, 0x1.2a436p+8);
b.cubicTo(0x1.000708p+9, 0x1.2a436p+8, 0x1.d2abdcp+7, 0x1.08b07cp+9, 0x1.8b2d9cp+9, 0x1.08b07cp+9);
b.quadTo(0x1.0a6ddcp+9, 0x1.3b8a1p+8, 0x1.0f0982p+9, 0x1.48e4c4p+8);
b.cubicTo(0x1.0f0982p+9, 0x1.48e4c4p+8, 0x1.3576cap+9, 0x1.b2cdc6p+7, 0x1.d1709cp+8, 0x1.b2cdc6p+7);
b.quadTo(0x1.1433dcp+9, 0x1.57ec68p+8, 0x1.13ed7p+9, 0x1.6340ap+8);
b.cubicTo(0x1.13ed7p+9, 0x1.6340ap+8, 0x1.107ddep+9, 0x1.ab9152p+7, 0x1.177e3cp+9, 0x1.ab9152p+7);
b.quadTo(0x1.13a424p+9, 0x1.6e9cc6p+8, 0x1.109c54p+9, 0x1.75c0f8p+8);
b.cubicTo(0x1.109c54p+9, 0x1.75c0f8p+8, 0x1.4feb3cp+9, 0x1.055c56p+9, 0x1.a25276p+8, 0x1.055c56p+9);
b.quadTo(0x1.0dcca8p+9, 0x1.7c5aap+8, 0x1.098a42p+9, 0x1.7f642cp+8);
b.cubicTo(0x1.098a42p+9, 0x1.7f642cp+8, 0x1.fe3e64p+9, 0x1.16801ap+9, 0x1.4ab5fp+5, 0x1.16801ap+9);
b.quadTo(0x1.05f5d4p+9, 0x1.81ede8p+8, 0x1.016b9ep+9, 0x1.82d6b6p+8);
b.cubicTo(0x1.016b9ep+9, 0x1.82d6b6p+8, 0x1.1361ecp+9, 0x1.865f68p+8, 0x1.dee448p+8, 0x1.865f68p+8);
b.quadTo(0x1.f8f24ep+8, 0x1.83cfb6p+8, 0x1.e9c828p+8, 0x1.7f2422p+8);
b.cubicTo(0x1.e9c828p+8, 0x1.7f2422p+8, 0x1.b55e48p+8, 0x1.8f5fdcp+8, 0x1.0f12eap+9, 0x1.8f5fdcp+8);
b.quadTo(0x1.dbd3dp+8, 0x1.7ad0bap+8, 0x1.ce8f2cp+8, 0x1.7471dep+8);
b.cubicTo(0x1.ce8f2cp+8, 0x1.7471dep+8, 0x1.48eb6ep+9, 0x1.165038p+8, 0x1.0b6fccp+8, 0x1.165038p+8);
b.quadTo(0x1.c0fc8ep+8, 0x1.6de5a4p+8, 0x1.b9918p+8, 0x1.681a6ap+8);
b.cubicTo(0x1.b9918p+8, 0x1.681a6ap+8, 0x1.82091ep+8, 0x1.939138p+8, 0x1.f10196p+8, 0x1.939138p+8);
b.quadTo(0x1.b24668p+8, 0x1.626298p+8, 0x1.aba974p+8, 0x1.5ae946p+8);
b.cubicTo(0x1.aba974p+8, 0x1.5ae946p+8, 0x1.96552p+8, 0x1.730aecp+8, 0x1.c0ef2ep+8, 0x1.730aecp+8);
b.quadTo(0x1.a4f65ep+8, 0x1.534faep+8, 0x1.9f6146p+8, 0x1.496e2cp+8);
b.cubicTo(0x1.9f6146p+8, 0x1.496e2cp+8, 0x1.1c7314p+9, 0x1.cb2f4p+5, 0x1.065becp+8, 0x1.cb2f4p+5);
b.quadTo(0x1.99e382p+8, 0x1.3fab16p+8, 0x1.960264p+8, 0x1.34972ap+8);
b.cubicTo(0x1.960264p+8, 0x1.34972ap+8, 0x1.a08be6p+8, 0x1.1676b4p+8, 0x1.8b85acp+8, 0x1.1676b4p+8);
b.quadTo(0x1.922156p+8, 0x1.297264p+8, 0x1.904476p+8, 0x1.1d2834p+8);
b.cubicTo(0x1.904476p+8, 0x1.1d2834p+8, 0x1.588a1ap+8, 0x1.4674d4p+9, 0x1.c74ep+8, 0x1.4674d4p+9);
b.quadTo(0x1.8e7184p+8, 0x1.10f81cp+8, 0x1.8e92fcp+8, 0x1.059f32p+8);
b.cubicTo(0x1.8e92fcp+8, 0x1.059f32p+8, 0x1.8d3546p+8, 0x1.1e1124p+7, 0x1.9029a8p+8, 0x1.1e1124p+7);
b.quadTo(0x1.8ebac4p+8, 0x1.f409bcp+7, 0x1.90875cp+8, 0x1.e40276p+7);
b.cubicTo(0x1.90875cp+8, 0x1.e40276p+7, 0x1.381e74p+8, -0x1.2f4f9p+7, 0x1.e9ae9p+8, -0x1.2f4f9p+7);
b.quadTo(0x1.925244p+8, 0x1.d42dcap+7, 0x1.94943p+8, 0x1.c9b692p+7);
b.cubicTo(0x1.94943p+8, 0x1.c9b692p+7, 0x1.071e6cp+9, 0x1.fe9cb4p+8, 0x1.1a65ccp+8, 0x1.fe9cb4p+8);
b.quadTo(0x1.96c65ap+8, 0x1.bf95c8p+7, 0x1.9c3eap+8, 0x1.adec18p+7);
b.cubicTo(0x1.9c3eap+8, 0x1.adec18p+7, 0x1.9f2694p+8, 0x1.a1a354p+7, 0x1.a551c8p+8, 0x1.a1a354p+7);
b.cubicTo(0x1.aa02f4p+8, 0x1.a7733ep+7, 0x1.aeb422p+8, 0x1.b9998ep+7, 0x1.b02756p+8, 0x1.b9998ep+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.a6df18p+8, 0x1.bb15fap+7);
b.quadTo(0x1.a1e182p+8, 0x1.cb3384p+7, 0x1.a00df2p+8, 0x1.d39ff4p+7);
b.lineTo(0x1.a00e6cp+8, 0x1.d39dbep+7);
b.quadTo(0x1.9e41dp+8, 0x1.dbf51ap+7, 0x1.9cb962p+8, 0x1.e97eb6p+7);
b.lineTo(0x1.9cb9a6p+8, 0x1.e97c5ep+7);
b.quadTo(0x1.9b3604p+8, 0x1.f6f99cp+7, 0x1.9b12c2p+8, 0x1.05c542p+8);
b.lineTo(0x1.9b12c6p+8, 0x1.05c41p+8);
b.quadTo(0x1.9af442p+8, 0x1.101c52p+8, 0x1.9ca082p+8, 0x1.1b49d6p+8);
b.lineTo(0x1.9ca054p+8, 0x1.1b48a8p+8);
b.quadTo(0x1.9e50d2p+8, 0x1.266e0ep+8, 0x1.a1cef4p+8, 0x1.3076a8p+8);
b.lineTo(0x1.a1ce8ep+8, 0x1.307588p+8);
b.quadTo(0x1.a5529ap+8, 0x1.3a7facp+8, 0x1.aa43f4p+8, 0x1.434926p+8);
b.lineTo(0x1.aa435ep+8, 0x1.43481cp+8);
b.quadTo(0x1.af33f8p+8, 0x1.4c0678p+8, 0x1.b506c8p+8, 0x1.52a1a6p+8);
b.lineTo(0x1.b505fep+8, 0x1.52a0c2p+8);
b.quadTo(0x1.bae1f8p+8, 0x1.593ffap+8, 0x1.c14426p+8, 0x1.5e4134p+8);
b.lineTo(0x1.c14338p+8, 0x1.5e4078p+8);
b.quadTo(0x1.c79feep+8, 0x1.63388cp+8, 0x1.d3f95p+8, 0x1.692daap+8);
b.lineTo(0x1.d3f83ep+8, 0x1.692d26p+8);
b.quadTo(0x1.e0686cp+8, 0x1.6f26p+8, 0x1.ed7722p+8, 0x1.73323ap+8);
b.lineTo(0x1.ed76p+8, 0x1.7331e2p+8);
b.quadTo(0x1.fa3966p+8, 0x1.772022p+8, 0x1.00cccp+9, 0x1.766686p+8);
b.lineTo(0x1.00cc2ap+9, 0x1.7666a4p+8);
b.quadTo(0x1.0495bap+9, 0x1.75a46ep+8, 0x1.077178p+9, 0x1.739da2p+8);
b.lineTo(0x1.0770ecp+9, 0x1.739e06p+8);
b.quadTo(0x1.0a2032p+9, 0x1.71b3fp+8, 0x1.0bd8d6p+9, 0x1.6da968p+8);
b.lineTo(0x1.0bd874p+9, 0x1.6daa4cp+8);
b.quadTo(0x1.0d8178p+9, 0x1.69c0c6p+8, 0x1.0daf5ap+9, 0x1.62a43cp+8);
b.lineTo(0x1.0daf52p+9, 0x1.62a568p+8);
b.quadTo(0x1.0ddb9p+9, 0x1.5b876cp+8, 0x1.09e45p+9, 0x1.4ffd58p+8);
b.lineTo(0x1.09e4a6p+9, 0x1.4ffe56p+8);
b.quadTo(0x1.05c626p+9, 0x1.440e54p+8, 0x1.f81558p+8, 0x1.33e43ap+8);
b.lineTo(0x1.f81644p+8, 0x1.33e4fep+8);
b.quadTo(0x1.e48036p+8, 0x1.23b0e6p+8, 0x1.cae7a6p+8, 0x1.157a82p+8);
b.lineTo(0x1.cae8b2p+8, 0x1.157b16p+8);
b.quadTo(0x1.b143ccp+8, 0x1.074ddep+8, 0x1.9d8bfp+8, 0x1.006d6ap+8);
b.lineTo(0x1.9d8d1p+8, 0x1.006dcep+8);
b.quadTo(0x1.89f336p+8, 0x1.f344a8p+7, 0x1.7fcc36p+8, 0x1.f235f4p+7);
b.lineTo(0x1.7fcd68p+8, 0x1.f23614p+7);
b.quadTo(0x1.75db02p+8, 0x1.f13668p+7, 0x1.7076ep+8, 0x1.f5af88p+7);
b.lineTo(0x1.7077fap+8, 0x1.f5ae9ep+7);
b.quadTo(0x1.6ad34ep+8, 0x1.fa639cp+7, 0x1.659ebap+8, 0x1.00f128p+8);
b.lineTo(0x1.659fbp+8, 0x1.00f076p+8);
b.quadTo(0x1.60214p+8, 0x1.04e8f6p+8, 0x1.5912e8p+8, 0x1.0962ep+8);
b.lineTo(0x1.5913eap+8, 0x1.09623cp+8);
b.quadTo(0x1.520ccp+8, 0x1.0ddc4cp+8, 0x1.45f9eep+8, 0x1.14e2d2p+8);
b.cubicTo(0x1.45f9eep+8, 0x1.14e2d2p+8, 0x1.44c8a2p+8, 0x1.152cd6p+8, 0x1.434088p+8, 0x1.152cd6p+8);
b.cubicTo(0x1.428ee2p+8, 0x1.13fb8ap+8, 0x1.41dd3ap+8, 0x1.114224p+8, 0x1.4244dep+8, 0x1.114224p+8);
b.quadTo(0x1.4f72cp+8, 0x1.0996e8p+8, 0x1.56643ap+8, 0x1.052aa8p+8);
b.cubicTo(0x1.56643ap+8, 0x1.052aa8p+8, 0x1.539b26p+8, 0x1.03651ep+8, 0x1.592f28p+8, 0x1.03651ep+8);
b.quadTo(0x1.5d52dcp+8, 0x1.00c4dap+8, 0x1.62b1e8p+8, 0x1.f9c618p+7);
b.cubicTo(0x1.62b1e8p+8, 0x1.f9c618p+7, 0x1.960618p+8, 0x1.21e2f2p+8, 0x1.2f4d1p+8, 0x1.21e2f2p+8);
b.quadTo(0x1.685ed6p+8, 0x1.f19a18p+7, 0x1.6e8b4ap+8, 0x1.ec73dap+7);
b.cubicTo(0x1.6e8b4ap+8, 0x1.ec73dap+7, 0x1.684f88p+8, 0x1.e745e2p+7, 0x1.74c96p+8, 0x1.e745e2p+7);
b.quadTo(0x1.74fb06p+8, 0x1.e71cbp+7, 0x1.800d94p+8, 0x1.e8394cp+7);
b.cubicTo(0x1.800d94p+8, 0x1.e8394cp+7, 0x1.29f0f6p+8, 0x1.f1314ap+7, 0x1.d6294ap+8, 0x1.f1314ap+7);
b.quadTo(0x1.8aeb02p+8, 0x1.e95bp+7, 0x1.9f305p+8, 0x1.f768d4p+7);
b.cubicTo(0x1.9f305p+8, 0x1.f768d4p+7, 0x1.0d1a98p+8, 0x1.2e9eep+8, 0x1.18976ep+9, 0x1.2e9eep+8);
b.quadTo(0x1.b34ffp+8, 0x1.02b90ep+8, 0x1.cd53fap+8, 0x1.111adep+8);
b.cubicTo(0x1.cd53fap+8, 0x1.111adep+8, 0x1.863a22p+8, 0x1.388bccp+8, 0x1.0a2e04p+9, 0x1.388bccp+8);
b.quadTo(0x1.e754fep+8, 0x1.1f8b42p+8, 0x1.fb463p+8, 0x1.300abep+8);
b.cubicTo(0x1.fb463p+8, 0x1.300abep+8, 0x1.b59828p+8, 0x1.69d224p+8, 0x1.206cbcp+9, 0x1.69d224p+8);
b.quadTo(0x1.07a2d6p+9, 0x1.40a638p+8, 0x1.0bf366p+9, 0x1.4d274ep+8);
b.cubicTo(0x1.0bf366p+9, 0x1.4d274ep+8, 0x1.158eb6p+9, 0x1.3147d6p+8, 0x1.025f18p+9, 0x1.3147d6p+8);
b.quadTo(0x1.106548p+9, 0x1.5a1638p+8, 0x1.102e92p+9, 0x1.62e38p+8);
b.cubicTo(0x1.102e92p+9, 0x1.62e38p+8, 0x1.0f52aep+9, 0x1.3f8582p+8, 0x1.1112c4p+9, 0x1.3f8582p+8);
b.quadTo(0x1.0ff5bcp+9, 0x1.6bb25ep+8, 0x1.0dc068p+9, 0x1.70e69p+8);
b.cubicTo(0x1.0dc068p+9, 0x1.70e69p+8, 0x1.1d9422p+9, 0x1.96247ep+8, 0x1.fbc742p+8, 0x1.96247ep+8);
b.quadTo(0x1.0b9862p+9, 0x1.75f69cp+8, 0x1.0847dcp+9, 0x1.7853aep+8);
b.cubicTo(0x1.0847dcp+9, 0x1.7853aep+8, 0x1.4574e4p+9, 0x1.a3babp+8, 0x1.962046p+8, 0x1.a3babp+8);
b.quadTo(0x1.052292p+9, 0x1.7a8e9ep+8, 0x1.010bf2p+9, 0x1.7b6044p+8);
b.cubicTo(0x1.010bf2p+9, 0x1.7b6044p+8, 0x1.058986p+9, 0x1.7c4272p+8, 0x1.f91b28p+8, 0x1.7c4272p+8);
b.quadTo(0x1.f9b68ep+8, 0x1.7c332ap+8, 0x1.ebfd44p+8, 0x1.77f92ep+8);
b.cubicTo(0x1.ebfd44p+8, 0x1.77f92ep+8, 0x1.dee2ccp+8, 0x1.7c081cp+8, 0x1.f914aep+8, 0x1.7c081cp+8);
b.quadTo(0x1.de936p+8, 0x1.73d0bp+8, 0x1.d1ce36p+8, 0x1.6daf0ap+8);
b.cubicTo(0x1.d1ce36p+8, 0x1.6daf0ap+8, 0x1.015012p+9, 0x1.5626ap+8, 0x1.a1065ep+8, 0x1.5626ap+8);
b.quadTo(0x1.c4f82ep+8, 0x1.677dc8p+8, 0x1.be2f54p+8, 0x1.62314p+8);
b.cubicTo(0x1.be2f54p+8, 0x1.62314p+8, 0x1.b04d3cp+8, 0x1.6d0ef4p+8, 0x1.cc0b5ap+8, 0x1.6d0ef4p+8);
b.quadTo(0x1.b7708cp+8, 0x1.5ce76ep+8, 0x1.b14762p+8, 0x1.55f0f6p+8);
b.cubicTo(0x1.b14762p+8, 0x1.55f0f6p+8, 0x1.abf24cp+8, 0x1.5bf96p+8, 0x1.b698dp+8, 0x1.5bf96p+8);
b.quadTo(0x1.ab1b54p+8, 0x1.4ef08cp+8, 0x1.a5e8eep+8, 0x1.45bdbcp+8);
b.cubicTo(0x1.a5e8eep+8, 0x1.45bdbcp+8, 0x1.cc4a26p+8, 0x1.01bbacp+8, 0x1.7fa798p+8, 0x1.01bbacp+8);
b.quadTo(0x1.a0bfc2p+8, 0x1.3c9108p+8, 0x1.9d167ep+8, 0x1.321c94p+8);
b.cubicTo(0x1.9d167ep+8, 0x1.321c94p+8, 0x1.9fb8dep+8, 0x1.2a9478p+8, 0x1.9a775p+8, 0x1.2a9478p+8);
b.quadTo(0x1.997106p+8, 0x1.27a2fep+8, 0x1.97aec8p+8, 0x1.1c0878p+8);
b.cubicTo(0x1.97aec8p+8, 0x1.1c0878p+8, 0x1.89c03p+8, 0x1.77f8d6p+8, 0x1.a5712ap+8, 0x1.77f8d6p+8);
b.quadTo(0x1.95f32ap+8, 0x1.10743ep+8, 0x1.9612dap+8, 0x1.05b55p+8);
b.cubicTo(0x1.9612dap+8, 0x1.05b55p+8, 0x1.95bb6ep+8, 0x1.d01f5p+7, 0x1.967886p+8, 0x1.d01f5p+7);
b.quadTo(0x1.9637ecp+8, 0x1.f5ccdcp+7, 0x1.97d8bap+8, 0x1.e74b9ap+7);
b.cubicTo(0x1.97d8bap+8, 0x1.e74b9ap+7, 0x1.81be8p+8, 0x1.227718p+7, 0x1.ae2288p+8, 0x1.227718p+7);
b.quadTo(0x1.997b98p+8, 0x1.d8d892p+7, 0x1.9b772p+8, 0x1.cfa7aep+7);
b.cubicTo(0x1.9b772p+8, 0x1.cfa7aep+7, 0x1.b9e14ap+8, 0x1.2e443p+8, 0x1.7ceb88p+8, 0x1.2e443p+8);
b.quadTo(0x1.9d703ep+8, 0x1.c68e04p+7, 0x1.a29ee8p+8, 0x1.b5d206p+7);
b.cubicTo(0x1.a29ee8p+8, 0x1.b5d206p+7, 0x1.a358e6p+8, 0x1.b2bfd6p+7, 0x1.a4e3b2p+8, 0x1.b2bfd6p+7);
b.cubicTo(0x1.a60ffep+8, 0x1.b433dp+7, 0x1.a73c48p+8, 0x1.b8bd64p+7, 0x1.a79916p+8, 0x1.b8bd64p+7);
b.close();
    testPathOp(reporter, left, b, kDifference_SkPathOp, filename);
}



static void pentrek11(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.a76d0cp+8, 0x1.902928p+7);
b.quadTo(0x1.9cb532p+8, 0x1.951a2ap+7, 0x1.99fec8p+8, 0x1.9909a6p+7);
b.quadTo(0x1.96954cp+8, 0x1.9dfe66p+7, 0x1.929578p+8, 0x1.a6c9d8p+7);
b.quadTo(0x1.8f2f3cp+8, 0x1.ae464ap+7, 0x1.8c92fp+8, 0x1.bac9e4p+7);
b.lineTo(0x1.8c9406p+8, 0x1.bac4acp+7);
b.quadTo(0x1.8a33d4p+8, 0x1.c6313ep+7, 0x1.8a584cp+8, 0x1.d5807ep+7);
b.lineTo(0x1.8a5838p+8, 0x1.d577fap+7);
b.quadTo(0x1.8a78d2p+8, 0x1.e2082ap+7, 0x1.9072e8p+8, 0x1.f2197cp+7);
b.lineTo(0x1.906fcap+8, 0x1.f21122p+7);
b.quadTo(0x1.9718eap+8, 0x1.01f2ccp+8, 0x1.a351b2p+8, 0x1.0a1974p+8);
b.lineTo(0x1.a34d42p+8, 0x1.0a167ep+8);
b.quadTo(0x1.af1d82p+8, 0x1.11ee58p+8, 0x1.ba4006p+8, 0x1.149b8p+8);
b.lineTo(0x1.ba383cp+8, 0x1.1499a4p+8);
b.quadTo(0x1.c4cd8ep+8, 0x1.171be6p+8, 0x1.d6f9d6p+8, 0x1.144a06p+8);
b.cubicTo(0x1.d6f9d6p+8, 0x1.144a06p+8, 0x1.dc6e08p+8, 0x1.172f06p+8, 0x1.e18b74p+8, 0x1.172f06p+8);
b.cubicTo(0x1.e26418p+8, 0x1.1ca338p+8, 0x1.e33cbcp+8, 0x1.2734d6p+8, 0x1.df7f18p+8, 0x1.2734d6p+8);
b.quadTo(0x1.c3fffep+8, 0x1.2b7912p+8, 0x1.b59b3cp+8, 0x1.280f94p+8);
b.cubicTo(0x1.b59b3cp+8, 0x1.280f94p+8, 0x1.f32e3ap+8, 0x1.1943bp+8, 0x1.780c3ap+8, 0x1.1943bp+8);
b.quadTo(0x1.a70036p+8, 0x1.248cdcp+8, 0x1.983d52p+8, 0x1.1acp+8);
b.cubicTo(0x1.983d52p+8, 0x1.1acp+8, 0x1.dc5df4p+8, 0x1.dabb4p+7, 0x1.54306cp+8, 0x1.dabb4p+7);
b.quadTo(0x1.8922cep+8, 0x1.10ad58p+8, 0x1.806a26p+8, 0x1.050106p+8);
b.cubicTo(0x1.806a26p+8, 0x1.050106p+8, 0x1.274c04p+9, -0x1.ec934p+3, 0x1.6591eep+7, -0x1.ec934p+3);
b.quadTo(0x1.769b16p+8, 0x1.efa396p+7, 0x1.765944p+8, 0x1.d6478ep+7);
b.cubicTo(0x1.765944p+8, 0x1.d6478ep+7, 0x1.7f7a2cp+8, -0x1.ae36p+7, 0x1.6df80ap+8, -0x1.ae36p+7);
b.quadTo(0x1.7620c4p+8, 0x1.be904ap+7, 0x1.7a1c9ep+8, 0x1.ab6838p+7);
b.cubicTo(0x1.7a1c9ep+8, 0x1.ab6838p+7, -0x1.14064ap+8, -0x1.5394ep+10, 0x1.027f48p+10, -0x1.5394ep+10);
b.quadTo(0x1.7e0e8p+8, 0x1.987f2ap+7, 0x1.83c9b8p+8, 0x1.8be00cp+7);
b.quadTo(0x1.892b4p+8, 0x1.800ac6p+7, 0x1.8e405cp+8, 0x1.78a8eap+7);
b.quadTo(0x1.944116p+8, 0x1.6ff35ap+7, 0x1.a2eef4p+8, 0x1.692ed8p+7);
b.cubicTo(0x1.a2eef4p+8, 0x1.692ed8p+7, 0x1.a84fcp+8, 0x1.6d6c52p+7, 0x1.adaf34p+8, 0x1.6d6c52p+7);
b.cubicTo(0x1.aeec94p+8, 0x1.782deap+7, 0x1.b029f4p+8, 0x1.8dae68p+7, 0x1.accdd6p+8, 0x1.8dae68p+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.a5bdc2p+8, 0x1.818b4ap+7);
b.quadTo(0x1.9989a8p+8, 0x1.872b9cp+7, 0x1.95976p+8, 0x1.8ce56p+7);
b.quadTo(0x1.918d88p+8, 0x1.92c30ap+7, 0x1.8d091p+8, 0x1.9cb22cp+7);
b.quadTo(0x1.88c2f6p+8, 0x1.a61b9ep+7, 0x1.85a6fap+8, 0x1.b5034ep+7);
b.lineTo(0x1.85a73ep+8, 0x1.b502p+7);
b.quadTo(0x1.82acaep+8, 0x1.c354e2p+7, 0x1.82d8a2p+8, 0x1.d5c7f2p+7);
b.lineTo(0x1.82d89cp+8, 0x1.d5c5d2p+7);
b.quadTo(0x1.8305acp+8, 0x1.e72272p+7, 0x1.8a6e74p+8, 0x1.fb0d9p+7);
b.lineTo(0x1.8a6dacp+8, 0x1.fb0b7ap+7);
b.quadTo(0x1.91dcap+8, 0x1.0778cp+8, 0x1.9f2864p+8, 0x1.1056ccp+8);
b.lineTo(0x1.9f2748p+8, 0x1.10561p+8);
b.quadTo(0x1.ac1286p+8, 0x1.18e9cap+8, 0x1.b87f4ep+8, 0x1.1be656p+8);
b.lineTo(0x1.b87d5cp+8, 0x1.1be5dep+8);
b.quadTo(0x1.c48078p+8, 0x1.1ebed6p+8, 0x1.d8203cp+8, 0x1.1bb352p+8);
b.cubicTo(0x1.d8203cp+8, 0x1.1bb352p+8, 0x1.d97d48p+8, 0x1.1c6c92p+8, 0x1.dac4a4p+8, 0x1.1c6c92p+8);
b.cubicTo(0x1.dafaccp+8, 0x1.1dc99ep+8, 0x1.db30f6p+8, 0x1.206e06p+8, 0x1.da418cp+8, 0x1.206e06p+8);
b.quadTo(0x1.c44d14p+8, 0x1.23d622p+8, 0x1.b7561cp+8, 0x1.20c35ap+8);
b.cubicTo(0x1.b7561cp+8, 0x1.20c35ap+8, 0x1.c6badcp+8, 0x1.1d1062p+8, 0x1.a7f25cp+8, 0x1.1d1062p+8);
b.quadTo(0x1.aa0b32p+8, 0x1.1d916ap+8, 0x1.9c634cp+8, 0x1.14807p+8);
b.cubicTo(0x1.9c634cp+8, 0x1.14807p+8, 0x1.ad6b74p+8, 0x1.0927d8p+8, 0x1.8b6012p+8, 0x1.0927d8p+8);
b.quadTo(0x1.8e5f18p+8, 0x1.0b2764p+8, 0x1.866c44p+8, 0x1.0083dap+8);
b.cubicTo(0x1.866c44p+8, 0x1.0083dap+8, 0x1.b9f7bcp+8, 0x1.76d4e4p+7, 0x1.5303f8p+8, 0x1.76d4e4p+7);
b.quadTo(0x1.7e0e3cp+8, 0x1.ea894ep+7, 0x1.7dd8ep+8, 0x1.d5f9b6p+7);
b.cubicTo(0x1.7dd8ep+8, 0x1.d5f9b6p+7, 0x1.80211ap+8, 0x1.e9b4a6p+6, 0x1.7bc09p+8, 0x1.e9b4a6p+6);
b.quadTo(0x1.7da7eap+8, 0x1.c16ca6p+7, 0x1.810966p+8, 0x1.b12ae4p+7);
b.cubicTo(0x1.810966p+8, 0x1.b12ae4p+7, 0x1.bb0156p+7, -0x1.60d8eap+7, 0x1.1280c4p+9, -0x1.60d8eap+7);
b.quadTo(0x1.847ac6p+8, 0x1.a0a9d6p+7, 0x1.89562p+8, 0x1.95f7b8p+7);
b.quadTo(0x1.8e3304p+8, 0x1.8b4622p+7, 0x1.92a7c4p+8, 0x1.84cd3p+7);
b.quadTo(0x1.976cap+8, 0x1.7de1e8p+7, 0x1.a49e3ep+8, 0x1.77ccb6p+7);
b.cubicTo(0x1.a49e3ep+8, 0x1.77ccb6p+7, 0x1.a5f67p+8, 0x1.78dc14p+7, 0x1.a74e4ep+8, 0x1.78dc14p+7);
b.cubicTo(0x1.a79da6p+8, 0x1.7b8c7ap+7, 0x1.a7ecfep+8, 0x1.80ec9ap+7, 0x1.a715f6p+8, 0x1.80ec9ap+7);
b.close();
    testPathOp(reporter, left, b, kDifference_SkPathOp, filename);
}

static void pentrek12(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
//b.setFillType(SkPathFillType::kEvenOdd);
b.moveTo(0x1.0f4bbcp+9, 0x1.2de136p+7);
b.quadTo(0x1.4b30b2p+8, 0x1.4c99eep+8, 0x1.3d8692p+8, 0x1.5e60b4p+8);
b.cubicTo(0x1.3d8692p+8, 0x1.5e60b4p+8, 0x1.3d2e1ep+8, 0x1.5e1308p+8, 0x1.3dc506p+8, 0x1.5e1308p+8);
b.quadTo(0x1.335046p+8, 0x1.6b3018p+8, 0x1.3daa2ap+8, 0x1.7d1fe4p+8);
b.cubicTo(0x1.3daa2ap+8, 0x1.7d1fe4p+8, 0x1.3d8acep+8, 0x1.7d798ep+8, 0x1.3ddc6ap+8, 0x1.7d798ep+8);
b.quadTo(0x1.4992a4p+8, 0x1.9285f2p+8, 0x1.c5c8a6p+8, 0x1.de872ap+8);
b.cubicTo(0x1.c5c8a6p+8, 0x1.de872ap+8, 0x1.ca7dd4p+8, 0x1.e791c8p+8, 0x1.cbf992p+8, 0x1.e791c8p+8);
b.cubicTo(0x1.c9181ep+8, 0x1.ec46f6p+8, 0x1.c636acp+8, 0x1.f277e2p+8, 0x1.c00d8p+8, 0x1.f277e2p+8);
b.quadTo(0x1.3aa6cp+8, 0x1.a0d706p+8, 0x1.2c43ep+8, 0x1.86fc3p+8);
b.lineTo(0x1.2c579ep+8, 0x1.871f08p+8);
b.quadTo(0x1.1b2d72p+8, 0x1.6960bp+8, 0x1.2dc75p+8, 0x1.520c8ep+8);
b.lineTo(0x1.2dab46p+8, 0x1.52306p+8);
b.quadTo(0x1.3c973ep+8, 0x1.3ec6e6p+8, 0x1.08c744p+9, 0x1.0f8acap+7);
b.cubicTo(0x1.08c744p+9, 0x1.0f8acap+7, 0x1.0adf12p+9, 0x1.094dep+7, 0x1.0e07e2p+9, 0x1.094dep+7);
b.cubicTo(0x1.0fd44ep+9, 0x1.11ad14p+7, 0x1.11a0bap+9, 0x1.26af84p+7, 0x1.116388p+9, 0x1.26af84p+7);
b.close();
SkPath left(b);
b.reset();
//b.setFillType(SkPathFillType::kEvenOdd);
b.moveTo(0x1.0cda0ep+9, 0x1.2280cep+7);
b.quadTo(0x1.45b726p+8, 0x1.476acap+8, 0x1.379456p+8, 0x1.59ce94p+8);
b.cubicTo(0x1.379456p+8, 0x1.59ce94p+8, 0x1.377e38p+8, 0x1.59bb2ap+8, 0x1.37a3f2p+8, 0x1.59bb2ap+8);
b.quadTo(0x1.2a4336p+8, 0x1.6a825p+8, 0x1.372b36p+8, 0x1.80df92p+8);
b.cubicTo(0x1.372b36p+8, 0x1.80df92p+8, 0x1.37235ep+8, 0x1.80f5fcp+8, 0x1.3737c6p+8, 0x1.80f5fcp+8);
b.quadTo(0x1.43fa2ep+8, 0x1.97e45ap+8, 0x1.c1de86p+8, 0x1.e4ece4p+8);
b.cubicTo(0x1.c1de86p+8, 0x1.e4ece4p+8, 0x1.c30bd2p+8, 0x1.e72f8cp+8, 0x1.c36ac2p+8, 0x1.e72f8cp+8);
b.cubicTo(0x1.c2b264p+8, 0x1.e85cd6p+8, 0x1.c1fa08p+8, 0x1.e9e912p+8, 0x1.c06fbcp+8, 0x1.e9e912p+8);
b.quadTo(0x1.403f36p+8, 0x1.9b789ep+8, 0x1.32d1a2p+8, 0x1.8356a4p+8);
b.lineTo(0x1.32d692p+8, 0x1.835f5ap+8);
b.quadTo(0x1.243a82p+8, 0x1.6a0e78p+8, 0x1.33a484p+8, 0x1.56b98cp+8);
b.lineTo(0x1.339d82p+8, 0x1.56c28p+8);
b.quadTo(0x1.4210cap+8, 0x1.43f60ap+8, 0x1.0b38f2p+9, 0x1.1aeb32p+7);
b.cubicTo(0x1.0b38f2p+9, 0x1.1aeb32p+7, 0x1.0bbee4p+9, 0x1.195bf8p+7, 0x1.0c8918p+9, 0x1.195bf8p+7);
b.cubicTo(0x1.0cfc34p+9, 0x1.1b73c4p+7, 0x1.0d6f4ep+9, 0x1.20b462p+7, 0x1.0d6002p+9, 0x1.20b462p+7);
b.close();
    testPathOp(reporter, left, b, kDifference_SkPathOp, filename);
}

static void pentrek13(skiatest::Reporter* reporter, const char* filename) {
SkPath b;
b.moveTo(0x1.1c44ccp+9, 0x1.bbff7p+7);
b.quadTo(0x1.b0e0f2p+8, 0x1.a86512p+7, 0x1.72767p+8, 0x1.6ba0f4p+7);
b.quadTo(0x1.576b86p+8, 0x1.514d0ep+7, 0x1.470454p+8, 0x1.2d2596p+7);
b.lineTo(0x1.4d6486p+8, 0x1.38f3e8p+7);
b.quadTo(0x1.55f3fp+8, 0x1.463cd4p+7, 0x1.6373fp+8, 0x1.47c2acp+7);
b.quadTo(0x1.724462p+8, 0x1.496e72p+7, 0x1.7f0c3cp+8, 0x1.39db6cp+7);
b.quadTo(0x1.8f7a0cp+8, 0x1.25d60cp+7, 0x1.94580ap+8, 0x1.0145cap+7);
b.quadTo(0x1.967178p+8, 0x1.e3014p+6, 0x1.95972cp+8, 0x1.c710dep+6);
b.quadTo(0x1.99a1bcp+8, 0x1.25bdb8p+7, 0x1.2bacd2p+9, 0x1.eb8efp+8);
b.cubicTo(0x1.2bacd2p+9, 0x1.eb8efp+8, 0x1.32540cp+9, 0x1.111518p+9, 0x1.2dea84p+9, 0x1.111518p+9);
b.cubicTo(0x1.21d408p+9, 0x1.17bc52p+9, 0x1.15bd8ap+9, 0x1.19fa04p+9, 0x1.068668p+9, 0x1.19fa04p+9);
b.quadTo(0x1.38b5bp+8, 0x1.645478p+7, 0x1.3253d8p+8, 0x1.f789e2p+6);
b.quadTo(0x1.30d6ccp+8, 0x1.c6c5p+6, 0x1.33b5bep+8, 0x1.9ba334p+6);
b.quadTo(0x1.392b24p+8, 0x1.499fep+6, 0x1.4b041p+8, 0x1.1e202p+6);
b.quadTo(0x1.58d18ap+8, 0x1.f8f8e6p+5, 0x1.69158p+8, 0x1.0027dp+6);
b.quadTo(0x1.7aaap+8, 0x1.041f24p+6, 0x1.8ab28ep+8, 0x1.35e32p+6);
b.cubicTo(0x1.8ab28ep+8, 0x1.35e32p+6, 0x1.8cfdbcp+8, 0x1.44e624p+6, 0x1.8f1f5p+8, 0x1.44e624p+6);
b.quadTo(0x1.94858ep+8, 0x1.5cb354p+6, 0x1.9e3b1p+8, 0x1.6f9af2p+6);
b.quadTo(0x1.cb502ap+8, 0x1.c762f4p+6, 0x1.1fdf34p+9, 0x1.e9092p+6);
b.cubicTo(0x1.1fdf34p+9, 0x1.e9092p+6, 0x1.2da248p+9, 0x1.286a88p+7, 0x1.37ffe8p+9, 0x1.286a88p+7);
b.cubicTo(0x1.37015cp+9, 0x1.5f76d2p+7, 0x1.3602dp+9, 0x1.bff9ap+7, 0x1.2a07dep+9, 0x1.bff9ap+7);
b.close();
SkPath left(b);
b.reset();
b.moveTo(0x1.1d9eb2p+9, 0x1.71315cp+7);
b.quadTo(0x1.8d1c3ep+8, 0x1.5807dap+7, 0x1.62c9bcp+8, 0x1.f57ee4p+6);
b.lineTo(0x1.6461cap+8, 0x1.fb660ep+6);
b.quadTo(0x1.63791p+8, 0x1.f893b6p+6, 0x1.62f9b8p+8, 0x1.f77772p+6);
b.quadTo(0x1.6309e6p+8, 0x1.f79b9p+6, 0x1.6345b4p+8, 0x1.f7f518p+6);
b.quadTo(0x1.64484cp+8, 0x1.f97836p+6, 0x1.659084p+8, 0x1.f9c244p+6);
b.quadTo(0x1.68b952p+8, 0x1.fa78bap+6, 0x1.6b892cp+8, 0x1.f39e54p+6);
b.quadTo(0x1.6f1c74p+8, 0x1.eae784p+6, 0x1.701b2ep+8, 0x1.dbf47p+6);
b.quadTo(0x1.70460ap+8, 0x1.d97092p+6, 0x1.704f2cp+8, 0x1.d7628ap+6);
b.quadTo(0x1.7050f6p+8, 0x1.d6fbcap+6, 0x1.7050d4p+8, 0x1.d6e722p+6);
b.quadTo(0x1.70523cp+8, 0x1.d7bf2ep+6, 0x1.705deep+8, 0x1.d93e54p+6);
b.quadTo(0x1.754938p+8, 0x1.3d364p+7, 0x1.1b3fb4p+9, 0x1.fda3fcp+8);
b.cubicTo(0x1.1b3fb4p+9, 0x1.fda3fcp+8, 0x1.1ce982p+9, 0x1.05a566p+9, 0x1.1bcf22p+9, 0x1.05a566p+9);
b.cubicTo(0x1.18c982p+9, 0x1.074f34p+9, 0x1.15c3e2p+9, 0x1.07dea2p+9, 0x1.11f61ap+9, 0x1.07dea2p+9);
b.quadTo(0x1.5d0e34p+8, 0x1.4cdbfp+7, 0x1.578d16p+8, 0x1.e55c6cp+6);
b.quadTo(0x1.575c6p+8, 0x1.df2042p+6, 0x1.5753p+8, 0x1.d982d6p+6);
b.quadTo(0x1.574b2ap+8, 0x1.d4d13ap+6, 0x1.575e94p+8, 0x1.d073bep+6);
b.quadTo(0x1.577fe6p+8, 0x1.c8f586p+6, 0x1.57f29ap+8, 0x1.c23a58p+6);
b.quadTo(0x1.5988bcp+8, 0x1.aa6474p+6, 0x1.5e872p+8, 0x1.9e38a4p+6);
b.quadTo(0x1.625c9ap+8, 0x1.94e09ep+6, 0x1.66f8ecp+8, 0x1.95eae4p+6);
b.quadTo(0x1.697c4p+8, 0x1.967c0ap+6, 0x1.6c0904p+8, 0x1.9a4d4p+6);
b.quadTo(0x1.6d9252p+8, 0x1.9c9a0cp+6, 0x1.6f2898p+8, 0x1.a024fap+6);
b.quadTo(0x1.714e6p+8, 0x1.a4f042p+6, 0x1.73b54ap+8, 0x1.ac64e2p+6);
b.cubicTo(0x1.73b54ap+8, 0x1.ac64e2p+6, 0x1.7447fep+8, 0x1.b026p+6, 0x1.74d09p+8, 0x1.b026p+6);
b.quadTo(0x1.98e5a6p+8, 0x1.279a82p+7, 0x1.1e854ep+9, 0x1.3f52a4p+7);
b.cubicTo(0x1.1e854ep+9, 0x1.3f52a4p+7, 0x1.21f612p+9, 0x1.4c4c22p+7, 0x1.248d7ap+9, 0x1.4c4c22p+7);
b.cubicTo(0x1.244dd8p+9, 0x1.5a0f34p+7, 0x1.240e34p+9, 0x1.722fe8p+7, 0x1.210f78p+9, 0x1.722fe8p+7);
b.close();
    testPathOp(reporter, left, b, kDifference_SkPathOp, filename);
}

void testQuadralaterals2582596(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
    path.setFillType(SkPathFillType::kEvenOdd);
    path.moveTo(0, 0);
    path.lineTo(2, 0);
    path.lineTo(1, 2);
    path.lineTo(2, 2);
    path.lineTo(0, 0);
    path.close();
    path.moveTo(0, 0);
    path.lineTo(1, 1);
    path.lineTo(0, 3);
    path.lineTo(3, 3);
    path.lineTo(0, 0);
    path.close();
    testSimplify(reporter, path, filename);
}

void testQuadralaterals19622648(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.lineTo(0, 1);
path.lineTo(1, 1);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 0);
path.lineTo(0, 1);
path.lineTo(3, 1);
path.lineTo(1, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void thread_cubics2665527(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.cubicTo(0, 2, 2, 0, 6, 4);
path.lineTo(0, 1);
path.close();
    SkPath pathB;
pathB.setFillType(SkPathFillType::kWinding);
pathB.moveTo(0, 2);
pathB.cubicTo(4, 6, 1, 0, 2, 0);
pathB.lineTo(0, 2);
pathB.close();
testPathOp(reporter, path, pathB, SkPathOp::kXOR_SkPathOp, filename);
}

void testQuads3759897(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(0, 1, 3, 1);
path.lineTo(0, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(2, 1, 1, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads3767769(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(0, 1, 3, 1);
path.lineTo(1, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(1, 0);
path.quadTo(1, 0, 1, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads5643714(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(0, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(0, 3, 2, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads7511735(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 0);
path.quadTo(2, 0, 0, 1);
path.lineTo(3, 2);
path.lineTo(1, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(1, 0, 0, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads1878310(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(1, 0, 3, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(0, 1);
path.quadTo(2, 1, 1, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads1878320(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(1, 0, 3, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(0, 1);
path.quadTo(3, 1, 1, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads1877944(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(1, 0, 3, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 0);
path.quadTo(1, 1, 1, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads5643583(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(0, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(0, 1, 1, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads5644589(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(0, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(1, 1);
path.quadTo(0, 2, 2, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads7544129(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 0);
path.quadTo(2, 0, 0, 1);
path.lineTo(3, 3);
path.lineTo(1, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 2);
path.quadTo(2, 2, 3, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads11296991(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 0);
path.quadTo(2, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(1, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(0, 1);
path.quadTo(2, 1, 2, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads18829957(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(3, 0);
path.quadTo(1, 2, 1, 2);
path.lineTo(0, 3);
path.lineTo(3, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(1, 0);
path.quadTo(3, 0, 0, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads3760641(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(0, 1, 3, 1);
path.lineTo(0, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 1);
path.quadTo(0, 1, 0, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads1892559(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(1, 0, 3, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(1, 1);
path.quadTo(1, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads18841185(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(3, 0);
path.quadTo(1, 2, 1, 2);
path.lineTo(1, 3);
path.lineTo(3, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(3, 1);
path.quadTo(1, 2, 1, 3);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads24458451(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 1);
path.quadTo(1, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(1, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(3, 0);
path.quadTo(0, 3, 3, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads17024527(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(3, 0);
path.quadTo(0, 1, 1, 2);
path.lineTo(1, 2);
path.lineTo(3, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(1, 1);
path.quadTo(0, 2, 3, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads11505991(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 0);
path.quadTo(0, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(1, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 0);
path.quadTo(0, 2, 2, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads1878027(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(1, 0, 3, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 0);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads11294741(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 0);
path.quadTo(2, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(1, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(2, 0, 1, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads28233681(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 2);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 2);
path.close();
path.moveTo(0, 0);
path.lineTo(3, 0);
path.quadTo(1, 3, 3, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads15233353(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 0);
path.quadTo(0, 2, 1, 3);
path.lineTo(1, 3);
path.lineTo(2, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 0);
path.quadTo(1, 2, 0, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads19022897(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(3, 0);
path.quadTo(1, 2, 3, 3);
path.lineTo(3, 3);
path.lineTo(3, 0);
path.close();
path.moveTo(0, 2);
path.lineTo(2, 2);
path.quadTo(3, 2, 3, 3);
path.lineTo(0, 2);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads13600555(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 0);
path.quadTo(0, 1, 3, 2);
path.lineTo(3, 2);
path.lineTo(2, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(1, 1);
path.quadTo(1, 1, 0, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads11271171(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 0);
path.quadTo(2, 2, 0, 3);
path.lineTo(3, 3);
path.lineTo(1, 0);
path.close();
path.moveTo(2, 2);
path.lineTo(2, 2);
path.quadTo(3, 2, 0, 3);
path.lineTo(2, 2);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads15062721(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 0);
path.quadTo(0, 2, 1, 2);
path.lineTo(1, 2);
path.lineTo(2, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 0);
path.quadTo(0, 1, 1, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

void testQuads26719853(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(3, 1);
path.quadTo(0, 2, 3, 2);
path.lineTo(3, 2);
path.lineTo(3, 1);
path.close();
path.moveTo(2, 1);
path.lineTo(3, 1);
path.quadTo(0, 2, 1, 3);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads26719853 had errors=172
void testQuads26750861(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(3, 1);
path.quadTo(0, 2, 3, 2);
path.lineTo(3, 3);
path.lineTo(3, 1);
path.close();
path.moveTo(2, 1);
path.lineTo(3, 1);
path.quadTo(0, 2, 1, 3);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads26750861 had errors=172
void testQuads23540861(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 1);
path.quadTo(3, 1, 1, 3);
path.lineTo(3, 3);
path.lineTo(1, 1);
path.close();
path.moveTo(1, 1);
path.lineTo(1, 1);
path.quadTo(2, 1, 2, 3);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads23540861 had errors=22
void testQuads3000789(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 0, 1, 1);
path.lineTo(3, 1);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(3, 0);
path.quadTo(0, 1, 0, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads3000789 had errors=66
void testQuads3002149(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 0, 1, 1);
path.lineTo(3, 1);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(3, 0);
path.quadTo(0, 1, 0, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads3002149 had errors=48
void testQuads3004179(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 0, 1, 1);
path.lineTo(3, 1);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(3, 0);
path.quadTo(0, 1, 0, 3);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads3004179 had errors=89
void testQuads29481893(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29481893 had errors=35

void testQuads29481894(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29481894 had errors=35
void testQuads29483253(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(1, 0);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29483253 had errors=35

void testQuads29483254(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(1, 0);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29483254 had errors=35
void testQuads29484373(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(2, 0);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29484373 had errors=35

void testQuads29484374(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(2, 0);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29484374 had errors=35
void testQuads29485283(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(3, 0);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29485283 had errors=33

void testQuads29485284(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(3, 0);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29485284 had errors=32
void testQuads29486011(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(0, 1);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29486011 had errors=35

void testQuads29486012(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 3);
path.lineTo(2, 2);
path.close();
path.moveTo(0, 1);
path.lineTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads29486012 had errors=35
void testQuads4970273(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(2, 1, 0, 3);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(0, 2, 1, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads4970273 had errors=77
void testQuads5008733(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(2, 1, 1, 3);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 1);
path.quadTo(1, 1, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5008733 had errors=48

void testQuads5008734(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(2, 1, 1, 3);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 1);
path.quadTo(1, 1, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5008734 had errors=48
void testQuads10622947(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 0);
path.quadTo(3, 1, 1, 3);
path.lineTo(2, 3);
path.lineTo(1, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(3, 1);
path.quadTo(1, 2, 2, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads10622947 had errors=80
void testQuads25814349(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 2);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25814349 had errors=31

void testQuads25814350(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 2);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25814350 had errors=31
void testQuads25822101(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 3);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25822101 had errors=24

void testQuads25822102(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 3);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25822102 had errors=31
void testQuads25829853(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 3);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25829853 had errors=24

void testQuads25829854(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 3);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25829854 had errors=31
void testQuads25837605(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 3);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25837605 had errors=24

void testQuads25837606(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 3);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25837606 had errors=31
void testQuads25845357(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 3);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25845357 had errors=16

void testQuads25845358(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 3);
path.lineTo(2, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads25845358 had errors=18
void testQuads5226111(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5226111 had errors=19

void testQuads5226112(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5226112 had errors=31
void testQuads5227471(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5227471 had errors=31

void testQuads5227472(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5227472 had errors=31
void testQuads5228591(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5228591 had errors=31

void testQuads5228592(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5228592 had errors=31
void testQuads5229501(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5229501 had errors=31

void testQuads5229502(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5229502 had errors=31
void testQuads5230230(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5230230 had errors=28
void testQuads5230801(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5230801 had errors=12

void testQuads5230802(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5230802 had errors=29

void testQuads5231241(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5231241 had errors=31

void testQuads5231242(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5231242 had errors=31
void testQuads5233864(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5233864 had errors=31
void testQuads5235224(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5235224 had errors=31
void testQuads5236344(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5236344 had errors=31
void testQuads5237254(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5237254 had errors=31
void testQuads5237982(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5237982 had errors=28
void testQuads5238554(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5238554 had errors=29

void testQuads5238994(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 2);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5238994 had errors=31
void testQuads5241615(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5241615 had errors=19

void testQuads5241616(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5241616 had errors=31
void testQuads5242975(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5242975 had errors=31

void testQuads5242976(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5242976 had errors=31
void testQuads5244095(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5244095 had errors=31

void testQuads5244096(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5244096 had errors=31
void testQuads5245005(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5245005 had errors=31

void testQuads5245006(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5245006 had errors=31
void testQuads5245734(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5245734 had errors=28
void testQuads5246305(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5246305 had errors=12

void testQuads5246306(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5246306 had errors=29
void testQuads5246745(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5246745 had errors=31

void testQuads5246746(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(0, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5246746 had errors=31
void testQuads5249367(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5249367 had errors=19

void testQuads5249368(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5249368 had errors=31
void testQuads5250727(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5250727 had errors=31

void testQuads5250728(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5250728 had errors=31
void testQuads5251847(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5251847 had errors=31

void testQuads5251848(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5251848 had errors=31
void testQuads5252757(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5252757 had errors=31

void testQuads5252758(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5252758 had errors=31
void testQuads5253486(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5253486 had errors=28

void testQuads5254057(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5254057 had errors=12

void testQuads5254058(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5254058 had errors=29

void testQuads5254497(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5254497 had errors=31

void testQuads5254498(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(1, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5254498 had errors=31
void testQuads5257119(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5257119 had errors=19

void testQuads5257120(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5257120 had errors=31
void testQuads5258479(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5258479 had errors=31

void testQuads5258480(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5258480 had errors=31
void testQuads5259599(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5259599 had errors=31

void testQuads5259600(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5259600 had errors=31
void testQuads5260509(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5260509 had errors=31

void testQuads5260510(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5260510 had errors=31
void testQuads5261238(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5261238 had errors=28
void testQuads5261809(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5261809 had errors=12

void testQuads5261810(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5261810 had errors=29

void testQuads5262249(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5262249 had errors=31

void testQuads5262250(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(2, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5262250 had errors=31
void testQuads5264871(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5264871 had errors=19

void testQuads5264872(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5264872 had errors=31
void testQuads5266231(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5266231 had errors=31

void testQuads5266232(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5266232 had errors=31
void testQuads5267351(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5267351 had errors=31

void testQuads5267352(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5267352 had errors=31
void testQuads5268261(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5268261 had errors=31

void testQuads5268262(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5268262 had errors=31
void testQuads5268990(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(0, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5268990 had errors=28
void testQuads5269561(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5269561 had errors=12

void testQuads5269562(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5269562 had errors=29

void testQuads5270001(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5270001 had errors=31

void testQuads5270002(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 0);
path.quadTo(3, 1, 2, 2);
path.lineTo(3, 3);
path.lineTo(0, 0);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 1);
path.quadTo(2, 2, 3, 2);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads5270002 had errors=31
void testQuads22094547(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 1, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(3, 1);
path.quadTo(1, 2, 2, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22094547 had errors=93
void testQuads22095907(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 1, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 0);
path.lineTo(3, 1);
path.quadTo(1, 2, 2, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22095907 had errors=97
void testQuads22097027(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 1, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 0);
path.lineTo(3, 1);
path.quadTo(1, 2, 2, 3);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22097027 had errors=97
void testQuads22098665(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 1, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 1);
path.lineTo(3, 1);
path.quadTo(1, 2, 2, 3);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22098665 had errors=95

void testQuads22099237(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 1, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 1);
path.lineTo(3, 1);
path.quadTo(1, 2, 2, 3);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22099237 had errors=92
void testQuads22099677(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 1, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 1);
path.lineTo(3, 1);
path.quadTo(1, 2, 2, 3);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22099677 had errors=97
void testQuads22102511(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22102511 had errors=35

void testQuads22102512(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22102512 had errors=35
void testQuads22103871(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22103871 had errors=32

void testQuads22103872(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22103872 had errors=32
void testQuads22104991(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22104991 had errors=32

void testQuads22104992(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22104992 had errors=32
void testQuads22105901(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22105901 had errors=32

void testQuads22105902(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22105902 had errors=32
void testQuads22106629(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22106629 had errors=29

void testQuads22106630(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22106630 had errors=29
void testQuads22107201(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22107201 had errors=35

void testQuads22107202(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22107202 had errors=35
void testQuads22107641(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22107641 had errors=32

void testQuads22107642(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22107642 had errors=32

void testQuads22107971(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(3, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22107971 had errors=32

void testQuads22107972(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(3, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22107972 had errors=32

void testQuads22108211(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 2);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(0, 2);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22108211 had errors=29

void testQuads22108212(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 2);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(0, 2);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22108212 had errors=32

void testQuads22108379(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 2);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(1, 2);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22108379 had errors=29

void testQuads22108380(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 2);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(1, 2);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22108380 had errors=32

void testQuads22108491(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 2);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(2, 2);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22108491 had errors=35

void testQuads22108492(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(2, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 2);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(2, 2);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22108492 had errors=35
void testQuads22110264(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(0, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22110264 had errors=32
void testQuads22111624(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22111624 had errors=29
void testQuads22112744(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22112744 had errors=29
void testQuads22113654(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(3, 0);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22113654 had errors=29
void testQuads22114382(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(0, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22114382 had errors=32
void testQuads22114954(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(1, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22114954 had errors=32
void testQuads22115394(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(2, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22115394 had errors=29

void testQuads22115724(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(3, 1);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(3, 1);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22115724 had errors=29

void testQuads22115964(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(0, 2);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(0, 2);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22115964 had errors=29

void testQuads22116132(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(1, 2);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(1, 2);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22116132 had errors=29

void testQuads22116244(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(0, 1);
path.quadTo(3, 2, 2, 3);
path.lineTo(3, 3);
path.lineTo(0, 1);
path.close();
path.moveTo(2, 2);
path.lineTo(2, 2);
path.quadTo(2, 3, 3, 3);
path.lineTo(2, 2);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads22116244 had errors=32
void testQuads14792779(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 0);
path.quadTo(3, 1, 1, 2);
path.lineTo(2, 2);
path.lineTo(2, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 0);
path.quadTo(1, 1, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads14792779 had errors=17

void testQuads14792780(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 0);
path.quadTo(3, 1, 1, 2);
path.lineTo(2, 2);
path.lineTo(2, 0);
path.close();
path.moveTo(1, 0);
path.lineTo(2, 0);
path.quadTo(1, 1, 3, 2);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads14792780 had errors=17
void testQuads14793899(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(2, 0);
path.quadTo(3, 1, 1, 2);
path.lineTo(2, 2);
path.lineTo(2, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 0);
path.quadTo(1, 1, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads14793899 had errors=74

void testQuads14793900(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kEvenOdd);
path.moveTo(2, 0);
path.quadTo(3, 1, 1, 2);
path.lineTo(2, 2);
path.lineTo(2, 0);
path.close();
path.moveTo(2, 0);
path.lineTo(2, 0);
path.quadTo(1, 1, 3, 2);
path.lineTo(2, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads14793900 had errors=74
void testQuads24281657(skiatest::Reporter* reporter, const char* filename) {
    SkPath path;
path.setFillType(SkPathFillType::kWinding);
path.moveTo(1, 1);
path.quadTo(3, 2, 0, 3);
path.lineTo(2, 3);
path.lineTo(1, 1);
path.close();
path.moveTo(1, 0);
path.lineTo(0, 1);
path.quadTo(3, 2, 0, 3);
path.lineTo(1, 0);
path.close();
    testSimplify(reporter, path, filename);
}

// testQuads24281657 had errors=252
// skia tests done: 560.873535s
// total run:30046752 skipped:0 errors:146 warnings:0 v0 only:0 skia only:10

static void (*skipTest)(skiatest::Reporter* , const char* filename) = nullptr;
static void (*firstTest)(skiatest::Reporter* , const char* filename) = nullptr;
static void (*stopTest)(skiatest::Reporter* , const char* filename) = nullptr;

#define TEST(name) { name, #name }

static struct TestDesc tests[] = {
    TEST(testQuads26719853),
    TEST(testQuads15062721),
    TEST(testQuads11271171),
    TEST(testQuads13600555),
    TEST(testQuads19022897),
    TEST(testQuads15233353),
    TEST(testQuads28233681),
    TEST(testQuads11294741),
    TEST(testQuads1878027),
    TEST(testQuads11505991),
    TEST(testQuads17024527),
    TEST(testQuads24458451),
    TEST(testQuads18841185),
    TEST(testQuads1892559),
    TEST(testQuads3760641),
    TEST(testQuads18829957),
    TEST(testQuads11296991),
    TEST(testQuads7544129),
    TEST(testQuads5644589),
    TEST(testQuads5643583),
    TEST(testQuads1877944),
    TEST(testQuads1878320),
    TEST(testQuads5643714),
    TEST(testQuads1878310),
    TEST(testQuads7511735),
    TEST(testQuads3767769),
    TEST(testQuads3759897),
    TEST(thread_cubics2665527),
    TEST(testQuadralaterals19622648),
    TEST(testQuadralaterals2582596),
    TEST(pentrek13),
    TEST(pentrek12),
    TEST(pentrek11),
    TEST(pentrek10),
    TEST(pentrek9),
    TEST(pentrek8),
    TEST(pentrek7),
    TEST(pentrek6),
    TEST(pentrek5),
    TEST(pentrek4),
    TEST(pentrek3),
    TEST(pentrek2),
    TEST(pentrek1),
    TEST(issue12556),
};

static const size_t testCount = std::size(tests);
static bool runReverse = false;

DEF_TEST(OpsV0, reporter) {
    RunTestSet(reporter, tests, testCount, firstTest, skipTest, stopTest, runReverse);
}

void run_v0_tests() {
    initializeTests(nullptr, "v0");
    test_OpsV0(nullptr);
}
