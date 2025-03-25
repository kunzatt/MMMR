package com.ssafy.mmmr.account.service;

import java.util.HashMap;
import java.util.Map;

import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import com.ssafy.mmmr.account.dto.SignUpReqeustDto;
import com.ssafy.mmmr.account.entity.AccountEntity;
import com.ssafy.mmmr.account.repository.AccountRepository;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.AccountException;
import com.ssafy.mmmr.global.error.exception.PasswordUpdateException;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;

@Service
@RequiredArgsConstructor
public class AccountService {

	private final AccountRepository accountRepository;
	private final PasswordEncoder passwordEncoder;

	@Transactional
	public void signUp(SignUpReqeustDto signUpReqeustDto) {
		// 이메일 중복 확인
		accountRepository.findByEmail(signUpReqeustDto.getEmail())
			.ifPresent(accountEntity -> {
				throw new AccountException(ErrorCode.EMAIL_EXIST);
			});

		// 계정 생성 및 저장
		AccountEntity account = AccountEntity.builder()
			.email(signUpReqeustDto.getEmail())
			.password(passwordEncoder.encode(signUpReqeustDto.getPassword()))
			.address(signUpReqeustDto.getAddress())
			.build();

		accountRepository.save(account);
	}

	public Map<String, Boolean> checkEmailExists(String email) {
		Map<String, Boolean> result = new HashMap<>();

		boolean exists = accountRepository.existsByEmail(email);
		result.put("exists", exists);

		return result;
	}

	@Transactional
	public void updatePassword(Long id, String currentPassword, String newPassword, String newPasswordConfirm) {
		if (id == null) {
			throw new AccountException(ErrorCode.ACCOUNT_NOT_FOUND);
		}

		if (currentPassword == null || newPassword == null || newPasswordConfirm == null) {
			throw new PasswordUpdateException(ErrorCode.INVALID_PASSWORD);
		}

		// 사용자 조회
		AccountEntity accountEntity = accountRepository.findById(id)
			.orElseThrow(() -> {
				return new AccountException(ErrorCode.ACCOUNT_NOT_FOUND);
			});

		// 현재 비밀번호 확인
		if (!passwordEncoder.matches(currentPassword, accountEntity.getPassword())) {
			throw new PasswordUpdateException(ErrorCode.INVALID_PASSWORD);
		}

		// 새 비밀번호 확인
		if (!newPassword.equals(newPasswordConfirm)) {
			throw new PasswordUpdateException(ErrorCode.PASSWORD_MISMATCH);
		}

		// 비밀번호 변경 및 저장
		accountEntity.changePassword(passwordEncoder.encode(newPassword));
		accountRepository.save(accountEntity);
	}
}