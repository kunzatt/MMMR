package com.ssafy.mmmr.account.service;

import java.time.Duration;
import java.security.SecureRandom;

import org.apache.commons.lang3.RandomStringUtils;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.data.redis.core.StringRedisTemplate;
import org.springframework.mail.javamail.JavaMailSender;
import org.springframework.mail.javamail.MimeMessageHelper;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;
import org.springframework.core.io.ClassPathResource;
import org.springframework.transaction.annotation.Transactional;
import org.thymeleaf.context.Context;
import org.thymeleaf.spring6.SpringTemplateEngine;

import com.ssafy.mmmr.account.entity.AccountEntity;
import com.ssafy.mmmr.account.repository.AccountRepository;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.AccountException;
import com.ssafy.mmmr.global.error.exception.EmailException;

import jakarta.mail.MessagingException;
import jakarta.mail.internet.MimeMessage;
import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class EmailService {

	private static final String EMAIL_CODE_PREFIX = "EMAIL_CODE:";
	private static final String VERIFIED_EMAIL_PREFIX = "VERIFIED_EMAIL:";
	private static final Duration CODE_EXPIRATION = Duration.ofMinutes(5);

	private final StringRedisTemplate redisTemplate;
	private final JavaMailSender mailSender;
	private final AccountRepository accountRepository;
	private final SpringTemplateEngine templateEngine;
	private final PasswordEncoder passwordEncoder;

	@Value("${spring.mail.username}")
	private String fromEmail;

	public void sendEmailCode(String email) {
		if (accountRepository.existsByEmail(email)) {
			throw new AccountException(ErrorCode.EMAIL_EXIST);
		}

		String code = RandomStringUtils.randomNumeric(6);
		saveVerificationCode(email, code);

		try {
			sendEmail(email, code);
		} catch (MessagingException e) {
			throw new EmailException(ErrorCode.EMAIL_SEND_FAIL);
		}
	}

	private void saveVerificationCode(String email, String code) {
		redisTemplate.opsForValue().set(
			EMAIL_CODE_PREFIX + email,
			code,
			CODE_EXPIRATION
		);
	}

	private void sendEmail(String to, String code) throws MessagingException {
		MimeMessage message = mailSender.createMimeMessage();
		MimeMessageHelper helper = new MimeMessageHelper(message, true, "UTF-8");

		Context context = new Context();
		context.setVariable("code", code);
		context.setVariable("expirationMinutes", CODE_EXPIRATION.toMinutes());

		String htmlContent = templateEngine.process("verification-code", context);

		helper.setFrom(fromEmail);
		helper.setTo(to);
		helper.setSubject("[MMMR] 이메일 인증번호");
		helper.setText(htmlContent, true);

		mailSender.send(message);
	}

	public void verifyCode(String email, String code) {
		String savedCode = redisTemplate.opsForValue().get(EMAIL_CODE_PREFIX + email);
		if (savedCode == null || !savedCode.equals(code)) {
			throw new EmailException(ErrorCode.INVALID_EMAIL_VERIFICATION);
		}
		redisTemplate.delete(EMAIL_CODE_PREFIX + email);
	}

	@Transactional
	public String sendTemporaryPassword(String email) {
		AccountEntity account = accountRepository.findByEmail(email)
			.orElseThrow(() -> new AccountException(ErrorCode.ACCOUNT_NOT_FOUND));

		String temporaryPassword = generateSecurePassword();

		account.changePassword(passwordEncoder.encode(temporaryPassword));
		accountRepository.save(account);

		try {
			sendPasswordResetEmail(email, temporaryPassword);
		} catch (MessagingException e) {
			throw new EmailException(ErrorCode.EMAIL_SEND_FAIL);
		}

		return temporaryPassword;
	}

	private String generateSecurePassword() {
		String LOWERCASE = "abcdefghijklmnopqrstuvwxyz";
		String UPPERCASE = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
		String NUMBERS = "0123456789";
		String SPECIAL = "!@#$%&?";

		SecureRandom random = new SecureRandom();

		StringBuilder password = new StringBuilder(10);

		password.append(LOWERCASE.charAt(random.nextInt(LOWERCASE.length())));
		password.append(UPPERCASE.charAt(random.nextInt(UPPERCASE.length())));
		password.append(NUMBERS.charAt(random.nextInt(NUMBERS.length())));
		password.append(SPECIAL.charAt(random.nextInt(SPECIAL.length())));

		String allChars = LOWERCASE + UPPERCASE + NUMBERS + SPECIAL;
		for (int i = 0; i < 6; i++) {
			password.append(allChars.charAt(random.nextInt(allChars.length())));
		}

		char[] passwordArray = password.toString().toCharArray();
		for (int i = 0; i < passwordArray.length; i++) {
			int randomIndex = random.nextInt(passwordArray.length);
			char temp = passwordArray[i];
			passwordArray[i] = passwordArray[randomIndex];
			passwordArray[randomIndex] = temp;
		}

		return new String(passwordArray);
	}

	private void sendPasswordResetEmail(String to, String temporaryPassword) throws MessagingException {
		MimeMessage message = mailSender.createMimeMessage();
		MimeMessageHelper helper = new MimeMessageHelper(message, true, "UTF-8");

		Context context = new Context();
		context.setVariable("temporaryPassword", temporaryPassword);

		String htmlContent = templateEngine.process("password-reset", context);

		helper.setFrom(fromEmail);
		helper.setTo(to);
		helper.setSubject("[MMMR] 임시 비밀번호 안내");
		helper.setText(htmlContent, true);

		mailSender.send(message);
	}
}