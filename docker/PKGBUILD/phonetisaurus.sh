# Maintainer: Jingbei Li <i@jingbei.li>
pkgname=phonetisaurus
pkgver=0.9.1
pkgrel=1
pkgdesc="WFST-driven grapheme-to-phoneme (g2p) framework suitable for rapid development of high quality g2p or p2g systems."
arch=('i686' 'x86_64' 'arm' 'armv6h' 'armv7h')
url="https://github.com/AdolfVonKleist/Phonetisaurus"
license=('BSD')
depends=('openfst' 'python')
makedepends=('python-pybindgen' 'python-setuptools' 'git')
source=("https://github.com/AdolfVonKleist/Phonetisaurus/archive/refs/tags/0.9.1.tar.gz")
sha256sums=('SKIP')
provides=('phonetisaurus')
conflicts=('phonetisaurus')

prepare() {
        cd "$srcdir/Phonetisaurus-$pkgver"
        sed '41ausing namespace std;' -i src/include/util.h
        #sed '/MapToken/s/string&/std::string&/g' -i src/include/util.h
}

build() {
        cd "$srcdir/Phonetisaurus-$pkgver"
        PYTHON=/usr/bin/python ./configure \
                --prefix=/usr \
                --with-openfst-includes=/usr/include/fst \
                --with-openfst-libs=/usr/lib \
                --enable-python
        make
}

package() {
        cd "$srcdir/Phonetisaurus-$pkgver/python"
        cp ../.libs/Phonetisaurus.so .
        python setup.py install --root="$pkgdir"/ --optimize=1

        cd "$srcdir/Phonetisaurus-$pkgver"
        make DESTDIR=${pkgdir} install

        install -Dm644 LICENSE "${pkgdir}/usr/share/licenses/${pkgname}/LICENSE"
}