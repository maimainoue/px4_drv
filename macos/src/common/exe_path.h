// exe_path.h - 実行ファイルのパスを返す (macOS と Linux で共通の呼び出し口)
#pragma once

#include <cstddef>
#include <cstdint>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#elif defined(__linux__)
#include <unistd.h>
#else
#error "exe_path.h: この OS の実行ファイルのパスの取得は未対応"
#endif

namespace px4 {

// 実行ファイルのパス (シンボリックリンクは解決しない) を buf へ入れる。成功で true。
// size は buf のバイト数で、終端の NUL を含めて収まらなければ false。
inline bool GetExecutablePath(char *buf, std::size_t size)
{
#if defined(__APPLE__)
	std::uint32_t path_size = static_cast<std::uint32_t>(size);

	return _NSGetExecutablePath(buf, &path_size) == 0;
#else
	// /proc/self/exe は終端の NUL を付けないため、size - 1 までを読む。
	ssize_t n;

	if (size < 2)
		return false;
	n = readlink("/proc/self/exe", buf, size - 1);
	if (n <= 0 || static_cast<std::size_t>(n) >= size - 1)
		return false;
	buf[n] = '\0';
	return true;
#endif
}

} // namespace px4
